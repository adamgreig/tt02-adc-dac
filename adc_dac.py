# Copyright 2022 Adam Greig
# ADC/DAC for TinyTapeout TT02

import numpy as np
import amaranth as am
from amaranth.back import verilog
from amaranth.sim import Simulator


class SDDAC(am.Elaboratable):
    """
    Sigma-delta DAC

    Generates pulse-density-modulated output where the average pulse
    density is controlled by the input data word. An external integrator
    (such as an RC circuit) is required to convert the pulse train
    to a DC voltage.

    Parameters:
        * `n`: Bit-width of input data

    Inputs:
        * `data`: Input data word to transmit, n bits wide

    Outputs:
        * `out`: Output 1-bit signal, pulse density modulated
    """
    def __init__(self, n):
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


class SDADC(am.Elaboratable):
    """
    Sigma-delta ADC

    Uses a sigma-delta DAC to generate an output voltage which is
    externally compared to the input analogue signal. A simple control
    loop adjusts the output voltage to track the input signal, and
    the control word for that DAC is the ADC reading.

    Parameters:
        * `n`: Bit-width of ADC (and of DAC control word)

    Inputs:
        * `comp`: Comparator result:
           1 if analogue signal is greater than DAC output, 0 otherwise

    Outputs:
        * `out`: Output 1-bit signal from DAC.
        * `data`: n-bit ADC reading
    """
    def __init__(self, n):
        self.n = n
        self.comp = am.Signal()
        self.out = am.Signal()
        self.data = am.Signal(n)

    def elaborate(self, platform):
        m = am.Module()

        dac = m.submodules.dac = SDDAC(self.n)
        m.d.comb += self.out.eq(dac.out), dac.data.eq(self.data)

        with m.If(self.comp):
            m.d.sync += self.data.eq(self.data - 1)
        with m.Else():
            m.d.sync += self.data.eq(self.data + 1)

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


class HexUartTx(am.Elaboratable):
    """
    Hex-coded UART transmitter.

    Transmits n-bit input words as hex-coded, newline-delimited ASCII.
    """
    def __init__(self, n):
        assert n % 4 == 0, "n must be a multiple of 4"
        self.n = n
        self.data = am.Signal(n)
        self.valid = am.Signal()
        self.ready = am.Signal()
        self.tx_o = am.Signal()

    def elaborate(self, platform):
        m = am.Module()
        n_nibbles = (self.n // 4)
        uart = m.submodules.uart = UartTx(n=8)
        m.d.comb += self.tx_o.eq(uart.tx_o)
        data_reg = am.Signal.like(self.data)
        nibble = am.Signal(4)
        with m.If(nibble < 10):
            m.d.comb += uart.data.eq(nibble + ord('0'))
        with m.Else():
            m.d.comb += uart.data.eq(nibble + ord('A') - 10)
        with m.FSM():
            with m.State("IDLE"):
                m.d.comb += self.ready.eq(uart.ready)
                m.d.sync += data_reg.eq(self.data)
                with m.If(self.valid):
                    m.next = "NIBBLE_0"
            for idx in range(n_nibbles):
                with m.State(f"NIBBLE_{idx}"):
                    word_idx = n_nibbles - idx - 1
                    m.d.comb += nibble.eq(data_reg.word_select(word_idx, 4))
                    m.d.comb += uart.valid.eq(1)
                    with m.If(uart.ready):
                        if idx == n_nibbles - 1:
                            m.next = "NEWLINE"
                        else:
                            m.next = f"NIBBLE_{idx + 1}"
            with m.State("NEWLINE"):
                m.d.comb += uart.data.eq(ord('\n'))
                m.d.comb += uart.valid.eq(1)
                with m.If(uart.ready):
                    m.next = "IDLE"
        return m


class Top(am.Elaboratable):
    def __init__(self):
        self.io_in = am.Signal(8)
        self.io_out = am.Signal(8)

    def elaborate(self, platform):
        m = am.Module()

        # Alias our inputs/outputs for convenience.
        clk_in = self.io_in[0]
        rst_in = self.io_in[1]
        adc_in = self.io_in[2]
        adc_out = self.io_out[0]
        uart_out = self.io_out[1]

        # Set up clock domain from io_in[0] and reset from io_in[1].
        cd_sync = am.ClockDomain("sync")
        m.d.comb += cd_sync.clk.eq(clk_in)
        m.d.comb += cd_sync.rst.eq(rst_in)
        m.domains += cd_sync

        # Create an ADC.
        adc = m.submodules.adc = SDADC(n=12)
        m.d.comb += adc.comp.eq(adc_in), adc_out.eq(adc.out)

        # Create a hex-coded UART for the ADC.
        adc_uart = m.submodules.adc_uart = HexUartTx(n=12)
        m.d.comb += uart_out.eq(adc_uart.tx_o), adc_uart.data.eq(adc.data)

        # Trigger the UART to transmit ten clock cycles after it becomes ready.
        ready_sr = am.Signal(10)
        m.d.sync += ready_sr.eq(am.Cat(adc_uart.ready, ready_sr))
        m.d.comb += adc_uart.valid.eq(ready_sr[-1])

        return m


def test_sddac():
    """Test SDDAC output bitstream has correct average value."""
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
    with sim.write_vcd("vcd/sddac.vcd"):
        sim.run()


def test_sdadc():
    """Test SDADC reports the correct voltage."""
    n = 10
    adc = SDADC(n)

    def testbench():
        rdgs = [(yield adc.out)]
        signal = 0.4
        for _ in range(1000):
            rdgs.append((yield adc.out))
            avg = np.mean(rdgs[-20:])
            yield adc.comp.eq(int(avg > signal))
            yield
        target = 2**n * 0.4
        reading = (yield adc.data)
        assert reading in range(int(target * 0.93), int(target * 1.08))

    sim = Simulator(adc)
    sim.add_clock(1/10e6)
    sim.add_sync_process(testbench)
    with sim.write_vcd("vcd/sdadc.vcd"):
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
    with sim.write_vcd("vcd/uart.vcd"):
        sim.run()


def test_hex_uart():
    """Test hex UART transmits a word correctly."""
    uart = HexUartTx(n=16)

    def testbench():
        yield
        assert (yield uart.ready)
        yield uart.data.eq(0xA2B3)
        yield uart.valid.eq(1)
        yield
        yield uart.data.eq(0)
        yield uart.valid.eq(0)
        yield
        yield
        bits = []
        for _ in range(54):
            bits.append((yield uart.tx_o))
            assert not (yield uart.ready)
            yield
        assert (yield uart.ready)
        expected_bits = []
        for byte in b"A2B3\n":
            expected_bits.append(0)
            for bit in range(8):
                expected_bits.append((byte >> bit) & 1)
            expected_bits.append(1)
            expected_bits.append(1)
        assert bits == expected_bits[:-1]
        yield

    sim = Simulator(uart)
    sim.add_clock(1/10e6)
    sim.add_sync_process(testbench)
    with sim.write_vcd("vcd/hex_uart.vcd"):
        sim.run()


def test_top():
    """
    Test overall Top design.

    1) ADC should output a hex-coded UART word that matches the input voltage.
    2) DAC should output a voltage controlled by the remaining inputs.
    """
    top = Top()
    uart_words = []

    def testbench_adc():
        # Simulates the external integrator and comparator.
        # Runs for 2000 cycles and then checks the latest hex word
        # matches the simulated input voltage.
        # Note we need to run for at least as many cycles as we expect the
        # output word to be, because it only changes by at most one count
        # per clock cycle.
        rdgs = [(yield top.io_out[0])]
        target = 0.4
        for _ in range(2000):
            rdgs.append((yield top.io_out[0]))
            avg = np.mean(rdgs[-20:])
            yield top.io_in[2].eq(int(avg > target))
            yield
        expected = 2**12 * target
        assert uart_words[-1] in range(int(expected*0.93), int(expected*1.08))

    def testbench_hex_uart():
        # Capture words received by the UART.
        yield am.sim.Passive()
        line = []
        while True:
            while (yield top.io_out[1]) == 1:
                yield
            bits = []
            for _ in range(9):
                bits.append((yield top.io_out[1]))
                yield
            byte = int("".join(str(b) for b in bits[1:])[::-1], 2)
            if chr(byte) != "\n":
                line.append(chr(byte))
            else:
                uart_words.append(int("".join(line), 16))
                line.clear()

    sim = Simulator(top)
    sim.add_clock(1/10e6)
    sim.add_sync_process(testbench_adc)
    sim.add_sync_process(testbench_hex_uart)
    with sim.write_vcd("vcd/top.vcd"):
        sim.run()


if __name__ == "__main__":
    top = Top()
    v = verilog.convert(
        top, name="adamgreig_tt02_adc_dac",
        ports=[top.io_out, top.io_in],
        emit_src=False, strip_internal_attrs=True)
    print(v)
