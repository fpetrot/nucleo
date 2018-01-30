# nucleo

Rabbits folder for nucleo folder. Currently only a part of the board STM32F4401RE is available. 

## Components implemented 

### GPIOs 

General Purpose In/Out are partly implemented. The currently available features are :
- select (write) or read the configuration mode
- select (write) or read the Output type
- select (write)  or read the Output speed
- write (and/or read) an Output data
- read an Input data
- choose between Floating, Pull-Up, Pull-Down and Analog input configuration
- bit set/reset of the Output data
- locking mecanism to freeze the configuration 

#### Conception

This component is based on a few registers : between others, there is one to store the configuration mode, one to store the Output data, and one to store the Input data. To access the two last, the configuration mode as to be set correctly (Input mode to read the Input register and Output mode to read the Output register).

### Timer 

General purpose timers are partly implemented. The features currently available are : 
- up or down auto-reload counter. up/down features is not yet implemented.
- a prescaler register used to divide counter clock.
- Output compare on 4 differents channels. 
- Interrupt generation for the following event : 
  - counter overflow/downflow
  - output compare matching counter 
  - manuel trigger by software 
  
Registers size for CNT, ARR and CCRx can be modified with the parameters "timer-size". 

As clock managing is not yet implemented, the current counter clock is 10ns.  

#### Conception 
 
This component is divided in two threads and its internal behaviour is based on shadow registers. Those shadow registers copy the content of their main register either during an update event or directly when their main register is updated by software and their preload bit is disabled ( bit ARPE for ARR register or OCxE for CCRx register).  

The "counter_thread" simulated the counter behaviour. It waits a amount of time based on the PSC register and its clock period ( currently 10ns ) and then decrease or increase CNT register according to DIR bit in CR1. If the counter overflow or underflow (ie CNT is above ARR or under 0 ), an update event is generated and the "irq_thread" is notified if needed. The same happened if a compare register matchs the counter. 

The "irq_thread" simply waits until `m_irq_update` event is notified. The irq port `p_irq` is then updated according to the irq status. This status is set either by the "counter_thread" or when the irq is acquited with SR register. 

### UART 

### Exti 

### RCC 

This component was made only to allow MBED based software to startup. Indead, Mbed fetch some values in RCC registers, Clock Frequency for example, and won't start if there are not found. Therefore, this component change registers values according to MBED boot but does nothing else.   
