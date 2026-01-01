# Bare-metal STM32F407 Environmental OLED Monitor for Temperature and Pressure
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

### 01/01/2025 Update
Completed enough of the I2C driver to run a scan for addresses and send the details over UART to view in RealTerm. Set up so not only all the scanned addresses will print but also the last known status on I2C. This has helped a lot in debugging as the OLED responded fine but the BMP280 module was not. After some time I realised pressing the module down on the breadboard allowed for a successful connection and the device repsponded when probed. I will need to sort out connections then finish the I2C driver.
