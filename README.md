# Nucleo

Rabbits folder for nucleo folder. Currently only a part of the board STM32F4401RE is available.

## Components implemented

To launch this simulation, you need to use `./nucleo-f401re.sh`. It will launch rabbits directly with F401RE platform. You need to provide a binary file. Because of internal conception, you must load it into two differents components ( see Flash  component for further information) by adding `-components.flash-native.file-blob file.bin -components.flash-boot.file-blob file.bin` to the previous command.

If you need to debug your software inside rabbits simulation. You can connect a gdb server to it. To do such thing, you need:
- To add `-gdb-server 1234` to the rabbits command.
- To run, in another terminal, `arm-none-eabi-gdb file.elf -ex "tar rem :1234"`, with `file.elf` your debugging file.

To create software, bare metal or mbed based software can be made. Mbed library provides Object-oriented classes as well as HAL functions which allow to easily control a component and its registers. Moreover, using mbed allows to run the same software either on the board or on rabbits siulation.
Mbed library can be downloaded there: https://os.mbed.com/users/mbed_official/code/mbed-dev/. However, as the FPU isn't simulated inside Qemu, all occurrences of `'-mfpu=fpv4-sp-d16' '-mfloat-abi=softfp'` must be removed in Makefile. Caution:  Normal mbed library cannot directly used as the precompile sources required this FPU. It must be recompiled, that's why "mbed-dev" is needed.


### Validation Method

Simulation validation was made with mbed-dev and its HAL functions. This method allows comparing rabbits behaviour with board one. Using object-oriented programs with MBED would have been too complicated. Those functions passed through several APIs before modifying registers. Moreover, those APIs, sometimes, employ only a part of the whole board capabilities. For example, all timing related functions rely only on timer 5, others are not used. Therefore, HAL functions allow full control on registers and then managing components functionalities as wanted.   

The main advantage with mbed is that it handles board's startup. Especially, it sets clock values which are needed by buses and their components. Inside Rabbits simulations, those clocks aren't managed yet. The component "RCC" was created only to handle this startup and provides values that mbed fetchs.

To launch a test on the Nucleo board, "st-link" must be installed. It provides `st-util` and `st-flash` binaries. This binary allows you to connect gdb to the board. To add a program on the board, you must use: `st-flash write path_to_test/test.bin 0x08000000 `

### Memory

This simulation has two differents memories, flash and sram. Currently, the flash stores the software binary and the sram is used as the software memory (stack, heap, ...) . However, as the board when booting fetchs the first instruction at 0x00000000, the flash needs to be aliased to this address. It can't be easily achieved inside rabbits. Therefore, we have created another components "flash-boot" which must contains the same binary than the default flash, component "flash-native". That's why this simulation needs twice the binary file. The flash interface which manipulates cache and provides acceleration, is currently disable. It is remplaced by a fake memory to prevent errors.

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

Defaults registers values can be passed via parameters for MODER, OSPEEDR and PUPDR.

#### Conception

This component checks that the mode configuration is correct before writing the Output Data Register (ODR). `gpio_thread` waits for an event happenning on this GPIO pins. When the event is notified, this thread will check the value pins and the corresponding data is written in the Input Data Register (IDR). Especially, this happens when ODR triggers a change on a pin, modifying IDR at the same time.

The writing to the Output Data Register was tested using the led component and the switch, to turn the led on or off when a key is pressed. The reading feature was tested using the led component and gpio-tester component, which alternatively (each 10 seconds, using `wait(10, SC_SEC)` in SystemC) switches a pin to true or false. The led state is updated depending on the pin value which is read. The result is that the led blinks with a period of 10 seconds.

### Timer

General purpose timers are partly implemented. The features currently available are:
- up or down auto-reload counter. up/down features is not yet implemented.
- a prescaler register used to divide counter clock.
- Output compare on 4 differents channels.
- Interrupt generation for the following event :
  - counter overflow/downflow
  - output compare matching counter
  - manual trigger by software using EGR register

Registers size for CNT, ARR and CCRx can be modified with the parameters "timer-size".

As clock managing is not yet implemented, the current counter clock is 10ns.  

#### Conception

This component is divided in two threads and its internal behaviour is based on shadow registers. Those shadow registers copy the content of their main register either during an update event or directly when their main register is updated by software and their preload bit is disabled ( bit ARPE for ARR register or OCxE for CCRx register).  

The `counter_thread` simulated the counter behaviour. It waits an amount of time based on the PSC register and its clock period ( currently 10ns ) and then decrease or increase CNT register according to DIR bit in CR1. If the counter overflow or underflow (ie CNT is above ARR or under 0 ), an update event is generated and the `irq_thread` is notified if needed. The same happened if a compare register matchs the counter.

The `irq_thread` simply waits until `m_irq_update` event is notified. The irq port `p_irq` is then updated according to the irq status. This status is set either by the `counter_thread` or when the irq is acquited with SR register.

Its validation was made inside test `timer_hal`. This test has two different parts. The first one validates the setup and the basic behaviour of a timer. It needs to trigger an IRQ which is redirected to `Basic_TIM_IRQHandler`function. This function simply acquits the interruption for this first part. For the second one, the timer is setup to trigger the same interrupt but only when CCR1 or CCR2 are matched by CNT registers. The IRQ hendler will then reset CCRx interrupt flags and put a new value in CCRx. Others behaviours like down-counting mode, clock division or shadow registers management were also tested but are no longer in the file.



### USART

Universal Synchronous Asynchronous Receiver Transmitter are 'almost' fully implemented.
The feature currently not available are :
- SmartCard NACK signal in emission. The NACK signal allow error detection and resending of data in case of parity error. The receiver, in case of parity error, must pull low the line during the transmission of stop bit to warn the transmitter of the error. But as in SmartCard mode the communication are half-duplex, the receiver must pull low the TX port of the USART. As the port in Rabbits are bind throw non-`SC_MANY_WRITTERS` signal, a NACK sending will stop the simulation due to error: `(E115) sc_signal<T> cannot have more than one driver.`
- Guard time before TC rise. In SmartCard mode the software can set a guard time in GTPR register to delay the rise of TC flag. This feature need to be implemented.
- LIN break character detection (LBD flag). This feature need to be implemented.
- IrDA facility. This model consider

#### Conception

This component is composed of four threads to handle the three main port of the USART, the communication input port RX, the communication output port TX, the clock for synchronous communication, and another one to handling the interrupt. The two hardware control flow port (nCTS and nRTS) are also drive by these threads.

##### read_thread
The reading thread is divide in 7 parts. The thread is composed of a infinite loop that execute, if it's necessary, these parts in this order:
- Mute mode: if mute mode is activated the thread will look for an idle frame to quit the mute mode. To detect the idle frame the thread will sample the line and count the number of high sample in a row. If the number of high sample is equal to an idle frame the thread quit the mute mode. The number of sample needed is determine from character length and the number of stop bit configured.
- Start bit detection: The thread look for the first start bit, that means a low edge on the reception line.
- Sampling data: the thread sample the data according to the configuration. Each bit can be sampled three time for noise or desynchronization detection, and the value applied is the mean value of the sample.
- Parity checking: the thread check the parity of the received frame, and update accordingly the status of the USART.
- Update data register. The thread will update the data register, if is empty, with the value of the receiving shifting register. Otherwise the data will not be update and the overrun flag will raise.
-  Stop bit sampling: the thread will sample the stop bit to validate the data, and update the status register. In case of SmartCard mode this part will also send a NACK signal when it's needed. But for now if it's happen the simulation will stop (cf unavailable feature).
-  Address mark detection wake up. If the receiver was mute with address mark detection wake up, this part need to compare the value of the data register with the address of the node.

##### send_thread

The sending thread is divide in 5 parts. The thread is composed of a infinite loop that execute each of these parts in this order:
- Waiting for transmission: firstly the thread will wait for a posedge on the control bit that enable the transmission. If a posedge is notify, the thread send an idle frame before handling the first data.
- Waiting data: The thread will wait for new data to send, for break character request or wait for the line in case of hardware flow control. It's in this part that the data register will be copy in the shifting register, and clear.
- Sending data: the thread send the data on the transmit line according to the configuration, and also create a synchronized clock if it's necessary, and adjust the parity bit in the MSB of data.
- Stop bit sending: the thread will send the stop bit, and sample the transmission line in case of SmartCard mode. In fact a NACK signal can be send by the receiver, and it need to be detect by the USART.
- Status update: the thread will update the status register, with the transmission complete flag, in case there is no new data to send. [Note: it's in this part the "Guard time before TX rise" should be implemented]

##### SCLK_thread

The SCLK port is a clock provide to the slave for synchronous or SmartCard mode. In case of synchronous mode, for example in SPI communication, the clock is composed at the same time as the data (cf send_thread). In SmartCard the SCLK signal is used by the SmartCard as internal clock, so SCLK is divided form internal system clock.
The thread will either applied the value calculate in send_thread, or divide the system clock, according to the configuration.

##### irq_update_thread

This thread will update the interrupt line of the USART. It need to be wake up after each possible change of a flag in the status register. The value of the interrupt line is a combinatorics combinations between status flag and interrupt enable control bit (xxIE).


### Exti / SYSCFG

Those two components are designed to trigger IRQ according to the GPIOs' pin values. Precisely, EXTI component generates interrupt requests according to a value modification on its input lines. These lines can be independently masked and configured ( rising and/or faling edge detection). SYSCFG components decided which GPIOs is connected to each one of the EXTI input lines.

At the moment, those components are created only for STM32F4xx board. Especially, the EXTI component has 6 Inport vectors. It isn't possible right now to create a vector of vectors of ports and make this component as generic as possible. Those vectors aims to be connected to each GPIO pins' vector.

#### Conception

EXTI component has several ports :
- `p_irq` is a Vector of OutPorts which aims to be connected to NVIC ports.
- `p_cfg` is a `InPort<uint32_t>` which receives messages for the SYSCFG component.
- 6 vectors of `InPort<bool>` vectors which are connected to gpios' pins.

Once this ports are created and connected, EXTI works on three threads :
- `irq_detection_thread` will wait until a GPIO's pin changed its values and find if an irq needs to be sent based on its registers. To detected a pin modification, a sc_event_or_list `m_ev_gpios` is linked to every pins connected. Therefore, even if a modification occurs inside a pin which shouldn't be connected to EXTI ( eg gpio-c is selected for irq 13 but a signal occurs on gpio-a), this thread will be wake up anyway. However, it searchs modification only on pins stored inside an array `m_gpios_selected`. So it won't trigger an interrupt for every pins modifications.
- `cfg_update_thread` will handle SYSCFG's messages and modify `m_gpios_selected` to react correctly. This thread wait until a modification occurs on `p_cfg` port. This port will then hhave a message as follows : 1 bit ( 17th ) give informations about which EXTICRx registers was changed inside SYSCFG, and the 16 lower bits will contains the new register values. This message is parsed by the thread and `m_gpios_selected` is update accordingly.
- `irq_update_thread` is used to generate irq when one is needed. It used `m_irq_status` to know which IRQ is set to true or to false.

SYSCFG is only composed of a register and sent messages to EXTI according to the previous rule when a change occurs.

### RCC

This component was made only to allow MBED based software to startup. Indeed, Mbed fetch some values in RCC registers, Clock Frequency for example, and won't start if there are not found. Therefore, this component changes its registers values according to MBED boot but does nothing else.   

### Test components

To ease tests and to provide better demonstrations, we have create some tests components. Some are really simple such as "led" which prints "led on" or "led off" according to its port value or "switch"  which changes its output value when a key is pressed. Some are more complex, such as the usart-tester. The operating is really similar than the USART itself, but it also provide trace file of the exchange. the file are named usartX-testerUsartTrace.vcd according to the number of the USART tested. These file record the modification of the signals, and can be open with viewer such as GTKWave for exemple. This allow a quick comparaison between the log trace, and the expected behavior.
