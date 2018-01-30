# Nucleo

Rabbits folder for nucleo folder. Currently only a part of the board STM32F401RE is available. 

To launch this simulation, you need to use `./nucleo-f401re.sh`. It will launch rabbits directly with F401RE platform. You need to provide a binary file. Because of internal conception, you must load it into two differents components ( see Flash for further information) by adding `-components.flash-native.file-blob file.bin -components.flash-boot.file-blob file.bin` to the previous command. 

If you need to debug your software inside rabbits simulation. You can connect a gdb server to it. To do such thing, you need: 
- To add `-gdb-server 1234` to the rabbits command. 
- To run, in another terminal, `arm-none-eabi-gdb file.elf -ex "tar rem :1234"`, with `file.elf` your debugging file. 

## Components implemented 

### Memory

This simulation has two differents memory, flash and sram. Currently, the flash stores the software binary and the sram is used as the software memory. However, as the board when booting fetch the first instruction at 0x00000000, we need to alias the flash to this address. It can't be easily achieved inside rabbits. Therefore, we have created another components "flash-boot" which must contains the same binary than the default flash, component "flash-native". The flash interface which manipulates cache and provides acceleration, is currently disable. It is remplaced by a fake memory to prevent errors. 

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

### RCC 

This component was made only to allow MBED based software to startup. Indead, Mbed fetch some values in RCC registers, Clock Frequency for example, and won't start if there are not found. Therefore, this component change registers values according to MBED boot but does nothing else.   

## Validation 

Most of the validation was made using mbed library. This library provides HAL functions which allows to easily control our components' registers. Moreover, using mbed allows us to run the same software either on the board or on rabbits siulation.
Mbed library can be downloaded their: https://os.mbed.com/users/mbed_official/code/mbed-dev/. However, as the FPU isn't simulated inside Qemu, in Maekfile, all occurences of `'-mfpu=fpv4-sp-d16' '-mfloat-abi=softfp'` must be removed. 

### Test with board

To launch a test on the Nucleo board, you must install "st-link" which provides "st-util" binary. This binary allows you to connect gdb to the board. 

To add a test on the board, `st-flash write path_to_test/test.bin 0x08000000 `

### Test components

To ease tests and to provide better demonstrations, we have create some tests components. Some are really simple such as "led" which prints "led on" or "led off" according to its port value or "switch"  which changes its output value when a key is pressed. 

UART-TESTER


