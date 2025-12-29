## This README will read as a project dairy

### Sprint 0 - Project Skeleton
This is where I set up folders I thought I would need, for example bsp/ drivers/ app/ utils/ etc.
I also implemented an error handling system that could come in handy throughout the project, with this come a logging interface that will be useful for debugging.

### Sprint 1 - Timebase and Heartbeat
Created a monotonic 1ms timebase that can be used for non-blocking scheduling, useful for sampling the temperature and pressure sensor and refreshing the OLED monitor.
Using the timebase I created a "heartbeat" blink using an onboard green LED, shows that CPU is functioning and also have an onboard red LED on standby as a warning light if needed.

### Sprint 2 - Setting up UART
Initialised a UART connection on GPIOA, using PA2 as TX and PA3 as RX. I connected this to a USB-UART adapter and successfully tested it using RealTerm. I set UART to have a baud rate of 115,200, 8 data bits, 1 stop bit and no hardware flow control.  
![image0 (1)](https://github.com/user-attachments/assets/effaff58-10db-499b-aad8-3342f08044e3)

