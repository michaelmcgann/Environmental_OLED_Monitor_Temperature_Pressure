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

### Sprint 3 - Wiring & I2C Driver

### 01/01/2026 Update
Completed enough of the I2C driver to run a scan for addresses and send the details over UART to view in RealTerm. Set up so not only all the scanned addresses will print but also the last known status on I2C. This has helped a lot in debugging as the OLED responded fine but the BMP280 module was not. After some time I realised pressing the module down on the breadboard allowed for a successful connection and the device responded when probed. I will need to sort out connections then finish the I2C driver.

### 01/01/2026 Update
Had to semi bodge the connections, but recieving both excpected ACKs now. Printed via UART on RealTerm as shown below. 

![image0 (2)](https://github.com/user-attachments/assets/4c31584c-1566-460e-b2bc-6b3b46ed88ef)

<img width="308" height="86" alt="image" src="https://github.com/user-attachments/assets/ad535d54-70aa-4067-add4-6c6e18e8494a" />

#### 07/01/2026 Update
Successfully read and printed the ID register from the BMP280 sensor. I now have a method that can read data from a register within a device on the bus. 

<img width="259" height="107" alt="image" src="https://github.com/user-attachments/assets/56f4b013-5824-47ff-b9cf-39ae391659a6" />
![image0 (3)](https://github.com/user-attachments/assets/c6ff878a-2048-4ee1-ae84-6dc851e9c0e6)


