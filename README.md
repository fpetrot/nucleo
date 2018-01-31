# Nucleo

<<<<<<< HEAD
Rabbits folder for nucleo folder. Currently only a part of the board STM32F4401RE is available.

## Components implemented
=======
Rabbits folder for nucleo platforms. Currently only a part of the board STM32F401RE is available. 

To launch this simulation, you need to use `./nucleo-f401re.sh`. It will launch rabbits directly with F401RE platform. You need to provide a binary file. Because of internal conception, you must load it into two differents components ( see Flash for further information) by adding `-components.flash-native.file-blob file.bin -components.flash-boot.file-blob file.bin` to the previous command. 

If you need to debug your software inside rabbits simulation. You can connect a gdb server to it. To do such thing, you need: 
- To add `-gdb-server 1234` to the rabbits command. 
- To run, in another terminal, `arm-none-eabi-gdb file.elf -ex "tar rem :1234"`, with `file.elf` your debugging file. 

To create software, bare metal or mbed based software can be made. Mbed library provides Object-oriented classes as well as HAL functions which allows to easily control a component and its registers. Moreover, using mbed allows to run the same software either on the board or on rabbits siulation. 
Mbed library can be downloaded their: https://os.mbed.com/users/mbed_official/code/mbed-dev/. However, as the FPU isn't simulated inside Qemu, all occurences of `'-mfpu=fpv4-sp-d16' '-mfloat-abi=softfp'` must be removed in Makefile. Caution :  Normal mbed library cannot directly used as the precompile sources required this FPU. It must be recompiled, that's why "mbed-dev" is needed. 

## Components implemented

### Validation Method

Simulation validation was made with mbed-dev and its HAL functions. This method allows to compare rabbits behaviour with board one. Using object-oriented programmation with MBED would have been to complicated. Those functions passed through several APIs before modifying registers. Moreover, those APIs, sometimes, employ only a part of the whole board capabilities. For example, all timing related functions rely only on timer 5, others are not used. Therefore, HAL functions allow to fully control registers and then managed components functionnalities as we want.   

The main advantage with mbed is that it handles board's startup. Especially, it sets clock values which are needed by bus and their components. Inside Rabbits simulations, those clocks aren't managed yet. The component "RCC" was create only to handle this startup and provides values that mbed fetchs. 

To launch a test on the Nucleo board, "st-link" must be installed. It provides `st-util` and `st-flash` binaries. This binary allows you to connect gdb to the board. To add a program on the board, you must use : `st-flash write path_to_test/test.bin 0x08000000 `

### Memory

This simulation has two differents memory, flash and sram. Currently, the flash stores the software binary and the sram is used as the software memory. However, as the board when booting fetch the first instruction at 0x00000000, we need to alias the flash to this address. It can't be easily achieved inside rabbits. Therefore, we have created another components "flash-boot" which must contains the same binary than the default flash, component "flash-native". The flash interface which manipulates cache and provides acceleration, is currently disable. It is remplaced by a fake memory to prevent errors. 
>>>>>>> 9c6f9cf119d5577a799d81311b5bb52b9a506ee1

### GPIOs

<<<<<<< HEAD
### Timer
=======
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

This components checks that the mode configuration is correct before writing or reading the Output Data Register or reading the Input Data Register. The gpio thread wait for an event to happen on the GPIOs. When the event is notified, for each pin of the GPIO port, the thread will check the value and the corresponding data is written in the Input Data Register.

#### Tests

The writing to the Output Data Register was tested using the led component and the switch, to turn the led on or off when a key is pressed. The reading feature was tested using the led component and gpio-tester component, which alternatively (each 10 seconds, using wait(10, SC_SEC) in SystemC) switches a pin to true or false. The led state is updated depending on the pin value which is read. The result is that the led blinks with a period of 10 seconds.

### Timer 
>>>>>>> 9c6f9cf119d5577a799d81311b5bb52b9a506ee1

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



<<<<<<< HEAD
### Exti
=======
### Exti / SYSCFG 

Those two components are designed to trigger IRQ according to the gpios' pin values. Precisely, EXTI component generates interrupt requests according to a value modification on its input lines. This lines can be independently masked and configured ( rising and/or faling edge detection). SYSCFG components decided which gpios is connected to each one of the EXTI input lines. 

#### Conception

At the moment, those components are created only for STM32F4xx board. Especially, EXTI component has 6 Inport vectors. It isn't possible right now to create a vector of vector of ports and make this component as generic as possible. It is also too complicated to connect differents ports to the same port. Therefore, linking EXTI's irq 5 to 9 to the same NVIC ports wasn't possible inside the yaml file. 

EXTI component has several ports : 
- p_irq is a Vector of OutPorts which aims to be connected to NVIC ports.
- p_cfg is a InPort<uint32_t> which receives messages for the SYSCFG component.
- 6 vectors of InPort<bool> vectors which are connected to gpios' pins. 

Once this ports are created and connected, EXTI works on three threads : 
- irq_detection_thread will wait until a gpios' pins changed its values and find if an irq need to be sent based on its registers. To detected a pin modification, a sc_event_or_list "m_ev_gpios" is linked to every pins connected. Therefore, even if a modification occurs inside a pin which shouldn't be connected to EXTI ( eg gpio-c is selected for irq 13 but a signal occurs on gpio-a), this thread will be wake up. However, it searchs modification only on pins stored inside an array 'm_gpios_selected'. So we won't trigger an interrupt every pins modifications.
- cfg_update_thread will handle SYSCFG's messages and modify "m_gpios_selected" to react correctly. This thread wait until a modification occurs on p_cfg port. This port will then a message as follows :1 bit ( 17th ) give informations about which EXTICRx registers was changed inside SYSCFG, and the 16 lower bits will contains the new register values. This message is parsed by the thread and m_gpios_selected is update accordingly.
- irq_update_thread is used to generate irq when one is needed. It used m_irq_status to know which irq is set to true or to false. At the moment, it handles the fix which connected the different IRQ to the same NVIC port. It simply redirects those irqs to the lowest of each one, 5 for 5 to 9 and 10 for 10 to 15. 

SYSCFG is only composed of a register and sent messages to EXTI according to the previous rule when a change occurs. 
>>>>>>> 9c6f9cf119d5577a799d81311b5bb52b9a506ee1

### RCC

This component was made only to allow MBED based software to startup. Indead, Mbed fetch some values in RCC registers, Clock Frequency for example, and won't start if there are not found. Therefore, this component change registers values according to MBED boot but does nothing else.   

### Test components

To ease tests and to provide better demonstrations, we have create some tests components. Some are really simple such as "led" which prints "led on" or "led off" according to its port value or "switch"  which changes its output value when a key is pressed. 
