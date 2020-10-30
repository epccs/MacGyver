# Analog-to-Digital Converter

from Microchip.
I added uart, to output channel scans.
added parts from the problem code, and now I have the problem

https://raw.githubusercontent.com/MicrochipTech/avr128da48-using-12-bit-adc/master/avr128da48-using-12-bit-adc/ADC_Single_Conversion

```
picocom -b 9600 /dev/ttyUSB0
...
Type [C-a] [C-h] to see available commands
Terminal ready
  1811    4095    3687    3296    2914    2545    2170    1812
  1809    4095    3687    3297    2914    2542    2174    1808
  1812    4095    3691    3295    2915    2542    2173    1811
  1809    4095    3687    3300    2914    2545    2171    1810
  1810    4095    3687    3298    2916    2542    2174    1809
  1812    4095    3689    3297    2915    2546    2171    1813
  1808    4094    3684    3298    2914    2543    2172    1809
  1812    4095    3689    3297    2916    2542    2174    1810
```

Channel zero has VDD, the other channels are floating.
This now shows the problem.