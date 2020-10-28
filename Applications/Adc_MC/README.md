# Analog-to-Digital Converter

from Microchip (I added uart, to output channel scans)

https://raw.githubusercontent.com/MicrochipTech/avr128da48-using-12-bit-adc/master/avr128da48-using-12-bit-adc/ADC_Single_Conversion

```
picocom -b 9600 /dev/ttyUSB0
...
Type [C-a] [C-h] to see available commands
Terminal ready
��ʊJ  2115        1850
  4095    4000    3618    3235    2851    2489    2119    1849
  4093    3999    3618    3233    2852    2488    2116    1850
  4094    3999    3616    3237    2850    2490    2115    1849
  4095    4002    3619    3236    2852    2490    2118    1848
  4095    3998    3619    3236    2852    2490    2116    1849
```

Channel zero has VDD, the other channels are floating, so this looks right.