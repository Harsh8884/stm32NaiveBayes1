# Implementation of NAIVE BAYES ML Algorithm on STM32
Here we used DSP Library to implement of stm32. In this project STM32F103C6T microcontroller is used , and it consist Cortex M3 processor.
In this project the microcontroller has 2 ADC channels to measure the output from the temperature sensor, and 1 UART for displaying the prediction.
NAIVE BAYES function is used to predict the wheather , there will be three classes and these classes are named as RAINY,SUNNY and WINTER.
The Parameter values used for the naive bayes function are obtained by python program, in which three datasets are trained and following parameters are obtained.
The ADC values are fed into the algorithm and there the naive bayes function takes place, and the output of the algorithm will be the prediction of the wheather.
Through UART the prediction can be displayed through serial monitor.
In this project the microcontroller performs the above algorithm every 10 ms.
Here the microcontroller is set to use 8MHZ clock configuration for low power consumption.

## Code Optimization for Low Power Consumption
* Use of Unsigned data type (ex- uint32_t,uint16_t...), it helps for memory management also.
* Use of Local variables than that of global variables.
* If in case, we use use multi ADC channels using DMA will help a lot. And we need to stop ADC sampling when it is performing other function.
* Usage of pointers.
* We used low minimal clock frequency, because we know that Higher the clock frequency Higher the power consumption.

 
