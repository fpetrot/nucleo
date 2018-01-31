# nucleo

Rabbits folder for nucleo folder. Currently only a part of the board STM32F4401RE is available.

## Components implemented

### GPIOs

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

### USART

Universal Synchronous Asynchronous Receiver Transmitter are 'almost' fully implemented.
The feature currently not available are :
- SmartCard NACK signal in emmission. The NACK signal allow error detection and resending of data in case of parity error. The receiver, in case of parity error, must pull low the line during the transmission of stop bit to warn the transmitter of the error. But as in smartcard mode the communication are half-duplex, the receiver must pull low the TX port of the usart. As the port in Rabbits are bind throw non-SC_MANY_WRITTERS signal, a NACK sending will stop the simulation due to error: (E115) sc_signal<T> cannot have more than one driver.
- Guard time before TC rise. In smartcard mode the software can set a guard time in GTPR register to delay the rise of TC flag. This feature need to be implemented.
- LIN break character detection (LBD flag). This feature need to be implemented.


#### Conception

This component is composed of four threads to handle the three main port of the USART, the communication input port RX, the communication outpout port TX, the clock for synchronous communication, and another one to handling the interrupt. The two hardware control flow port (nCTS and nRTS) are also drive by these threads.

##### read_thread
The reading thread is divide in X parts. The thread is composed of a infinite loop that execute each of these parts in this order:
- Mute mode: if mute mode is activated the thread will look for an idle frame to quit the mute mode. To detect the idle frame the thread will sample the line and count the number of high sample in a row. If the number of high sample is equal to an idle frame the thread quit the mute mode. The number of sample needed is determine from character length and the number of stop bit configured.
- Start bit detection: The thread look for the first start bit, that means a low edge on the reception line.
- Sampling data: the thread sample the data according to the configuration. Each bit can be sampled three time for noise or desynchonization detection, and the value applied is the mean value of the sample.
- Parity checking: the thread check the parity of the received frame, and update accordingly the status of the usart.
- Update data register. The thread will update the data register, if is empty, with the value of the receiving shifting register. Otherwise the data will not be update and the overrun flag will raise.
-  Stop bit sampling: the thread will sample the stop bit to validate the data, and update the status register. In case of smartcard mode this part will also send a NACK signal when it's needed. But for now if it's happen the simulation will stop (cf unavailable feature).
-  Address mark detetion wake up. This part need to compare the value of the data register  detect the address of the node in case of address mark wake up configuration.

##### send_thread


##### SCLK_thread


##### irq_update_thread



### Exti

### RCC

This component was made only to allow MBED based software to startup. Indead, Mbed fetch some values in RCC registers, Clock Frequency for example, and won't start if there are not found. Therefore, this component change registers values according to MBED boot but does nothing else.   
