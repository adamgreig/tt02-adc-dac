.PHONY: all test

all: src/adc_dac.v

src/adc_dac.v: adc_dac.py
	python3 adc_dac.py > src/adc_dac.v

test: adc_dac.py
	pytest adc_dac.py
