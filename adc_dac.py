# Copyright 2022 Adam Greig
# ADC/DAC for TinyTapeout TT02

import amaranth as am
from amaranth.back import verilog
from amaranth.sim import Simulator


class SDDAC(am.Elaboratable):
    """
    Sigma-delta DAC

    Parameters:
        * `n`: Bit-width of input data

    Inputs:
        * `data`: Input data word to transmit, n bits wide

    Outputs:
        * `out`: Output 1-bit signal, pulse density modulated
    """
    def __init__(self, n=8):
        self.n = n
        self.data = am.Signal(n)
        self.out = am.Signal()

    def elaborate(self, platform):
        m = am.Module()

        # Compact first-order sigma-delta modulator.
        # The output feedback stage is implemented by removing the MSbit
        # from the accumulator feedback, with that bit used for output.
        acc = am.Signal(self.n+1)
        m.d.sync += acc.eq(acc[:-1] + self.data)
        m.d.comb += self.out.eq(acc[-1])

        return m


class UartTx(am.Elaboratable):
    """
    Basic UART transmitter.

    Transmits 8n1 data at the sync clock rate.

    Inputs:
        * `data`: n-bit input data word to transmit
        * `valid`: Pulse high while ready to trigger transmission

    Outputs:
        * `ready`: High while idle
        * `tx_o`: Output signal
    """
    def __init__(self, n=8):
        self.data = am.Signal(n)
        self.valid = am.Signal()

        self.ready = am.Signal()
        self.tx_o = am.Signal()

        self.n = n

    def elaborate(self, platform):
        m = am.Module()

        tx_reg = am.Signal(self.n+2, reset=1)
        tx_cnt = am.Signal(range(self.n+3))

        m.d.comb += self.tx_o.eq(tx_reg[0])

        with m.If(tx_cnt == 0):
            # Idle
            m.d.comb += self.ready.eq(1)
            with m.If(self.valid):
                m.d.sync += [
                    tx_reg.eq(am.Cat(0, self.data, 1)),
                    tx_cnt.eq(self.n+2),
                ]

        with m.Else():
            # Transmitting
            m.d.comb += self.ready.eq(0)
            # Update output state
            m.d.sync += [
                tx_reg.eq(am.Cat(tx_reg[1:], 1)),
                tx_cnt.eq(tx_cnt - 1),
            ]

        return m


class Top(am.Elaboratable):
    def __init__(self):
        self.io_in = am.Signal(8)
        self.io_out = am.Signal(8)

    def elaborate(self, platform):
        m = am.Module()

        # Set up clock domain from io_in[0] and reset from io_in[1].
        cd_sync = am.ClockDomain("sync")
        m.d.comb += cd_sync.clk.eq(self.io_in[0])
        m.d.comb += cd_sync.rst.eq(self.io_in[1])
        m.domains += cd_sync

        # Call in[2] the ADC input, out[0] the DAC output, out[1] the UART Tx.
        adc_in = self.io_in[2]
        dac_out = self.io_out[0]
        uart_txo = self.io_out[1]

        # Use the other five inputs as a parallel DAC input for now.
        # When they're all 0, the DAC is used for the ADC.
        dac_ctrl = self.io_in[3:]

        # Accumulate to track the input signal.
        acc = am.Signal(8)
        with m.If(adc_in):
            m.d.sync += acc.eq(acc - 1)
        with m.Else():
            m.d.sync += acc.eq(acc + 1)

        # Wire up to DAC and UART.
        dac = m.submodules.dac = SDDAC()
        uart_tx = m.submodules.uart_tx = UartTx()
        m.d.comb += [
            dac.data.eq(am.Mux(dac_ctrl == 0, acc, dac_ctrl << 3)),
            dac_out.eq(dac.out),
            uart_tx.data.eq(acc),
            uart_txo.eq(uart_tx.tx_o),
        ]

        # Send a new word ten clocks after becoming ready.
        ready_sr = am.Signal(10)
        m.d.sync += ready_sr.eq(am.Cat(uart_tx.ready, ready_sr))
        m.d.comb += uart_tx.valid.eq(ready_sr[-1])

        return m


def test_sddac():
    """Test SDDAC output bitstream has correct average value."""
    import numpy as np
    sddac = SDDAC(n=8)

    def testbench():
        for x in (0, 63, 127, 255):
            yield sddac.data.eq(x)
            signal = np.empty(1000)
            for t in range(signal.size):
                yield
                signal[t] = (yield sddac.out)
            assert np.abs(np.mean(signal) - x/256) < 0.005

    sim = Simulator(sddac)
    sim.add_clock(1/10e6)
    sim.add_sync_process(testbench)
    sim.run()


def test_uart():
    """Test UART transmits a word correctly."""
    uart = UartTx()

    def testbench():
        yield
        assert (yield uart.ready)
        yield uart.data.eq(0xA2)
        yield uart.valid.eq(1)
        yield
        yield uart.data.eq(0)
        yield uart.valid.eq(0)
        yield
        bits = []
        for _ in range(10):
            bits.append((yield uart.tx_o))
            assert not (yield uart.ready)
            yield
        assert (yield uart.ready)
        assert bits == [0, 0, 1, 0, 0, 0, 1, 0, 1, 1]
        yield
        assert (yield uart.tx_o)

    sim = Simulator(uart)
    sim.add_clock(1/10e6)
    sim.add_sync_process(testbench)
    sim.run()


def test_top():
    """Test Top converts a "voltage" to a UART word."""
    import numpy as np
    top = Top()
    uart_words = []

    def testbench_adc():
        rdgs = [(yield top.io_out[0])]
        target = 0.4
        for _ in range(1000):
            rdgs.append((yield top.io_out[0]))
            avg = np.mean(rdgs[-20:])
            yield top.io_in[2].eq(int(avg > target))
            yield
        assert uart_words[-1] in range(107, 112)

    def testbench_uart():
        yield am.sim.Passive()
        while True:
            while (yield top.io_out[1]) == 1:
                yield
            bits = []
            for _ in range(9):
                bits.append((yield top.io_out[1]))
                yield
            word = int("".join(str(b) for b in bits[1:])[::-1], 2)
            uart_words.append(word)

    sim = Simulator(top)
    sim.add_clock(1/10e6)
    sim.add_sync_process(testbench_adc)
    sim.add_sync_process(testbench_uart)
    with sim.write_vcd("top.vcd"):
        sim.run()


if __name__ == "__main__":
    top = Top()
    v = verilog.convert(
        top, name="adamgreig_tt02_adc_dac",
        ports=[top.io_out, top.io_in],
        emit_src=False, strip_internal_attrs=True)
    print(v)
