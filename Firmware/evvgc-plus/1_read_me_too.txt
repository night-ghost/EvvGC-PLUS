LOAD - swd
LOAD - serial, use buttons

suspicious: USB _... WAIT in halconf - in MAPLE false, here true


maple mini works OK:

ch> 
ch> info
Kernel:       4.0.0
Compiler:     GCC 4.9.3 20150529 (release) [ARM/embedded-4_9-branch revision 224288]
Architecture: ARMv7-M
Core Variant: Cortex-M3
Port Info:    Advanced kernel mode
Platform:     STM32F10x Performance Line Medium Density
Board:        LeafLabs Maple Mini
Build time:   Oct 29 2016 - 13:23:22
ch> 

our gimbal plate with the problem:

ch> info
Kernel:       3.1.4
Compiler:     GCC 4.9.3 20150529 (release) [ARM/embedded-4_9-branch revision 224288]
Architecture: ARMv7-M
Core Variant: Cortex-M3
Port Info:    Advanced kernel mode
Platform:     STM32F10x Performance Line High Density
Board:        Storm32 v1.31
Build time:   Oct 30 2016 - 17:53:14
ch> 
ch> 

two differences:

no directive _RT_ in chconf.h

our Makefile:

include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f1xx.mk

in MAPLE (this good, i.e. m/PROGRAMMING/stm32/ChibiOS (what is? 16.1.5)

include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f1xx.mk

first, let's convert this maple to our board
