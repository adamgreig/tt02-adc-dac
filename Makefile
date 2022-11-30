.PHONY: all

all: adc_dac.py
	python3 adc_dac.py > src/adc_dac.v
