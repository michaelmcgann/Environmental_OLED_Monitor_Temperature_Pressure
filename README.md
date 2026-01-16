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
Had to semi bodge the connections, but receiving both expected ACKs now. Printed via UART on RealTerm as shown below. 

![image0 (2)](https://github.com/user-attachments/assets/4c31584c-1566-460e-b2bc-6b3b46ed88ef)

<img width="308" height="86" alt="image" src="https://github.com/user-attachments/assets/ad535d54-70aa-4067-add4-6c6e18e8494a" />

### Sprint 4 - Finish drivers for BMP280 and OLED

#### 07/01/2026 Update
Successfully read and printed the ID register from the BMP280 sensor. I now have a method that can read data from a register within a device on the bus. 

<img width="259" height="107" alt="image" src="https://github.com/user-attachments/assets/56f4b013-5824-47ff-b9cf-39ae391659a6" />

![image0 (3)](https://github.com/user-attachments/assets/c6ff878a-2048-4ee1-ae84-6dc851e9c0e6)

### 12/01/2028 Update
I am working my way through developing the BMP280 driver, I was doing some testing and realised I was getting an I2C timeout error on every write. Using my UART debug logger I managed to zone in on the error, if I didn't laugh I would cry as it took me longer than I care to admit! Screenshot of the error below, I am sure you will see it immediately. My I2C driver is over 800 lines of code long so that is my excuse! 😂 

<img width="521" height="133" alt="image" src="https://github.com/user-attachments/assets/9cefb6ba-1b0a-4af5-b93d-372553f540b7" />

Fixed below:

<img width="578" height="148" alt="image" src="https://github.com/user-attachments/assets/fc4512a3-f580-4e58-bd06-259f2939eafb" />

#### Update 14/01/2026

Measurements taken from raw values before compensate seem to be reading fine and changing with each test meaning changes in temp and pressure are being successfully read from registers

<img width="458" height="308" alt="image" src="https://github.com/user-attachments/assets/8bbae2c5-03fe-484f-9759-970d3704407d" />

#### Update 14/01/2026

Compensate function working well, tried and tested with results formatted and printed over UART. Figures for temperature and pressure look believable. The final stages are now to create the OLED driver and bring it all together in a loop while taking advantage of our timebase.

<img width="294" height="193" alt="image" src="https://github.com/user-attachments/assets/e365f771-ebb4-4a08-8229-86c8e9c43fe3" />

#### Update 15/01/2026

Having a read through the datasheet of the OLED SSD1306 screen to try and decipher how to place characters to display on it. There are libraries for this but it will be beneficial to at least once go through the manual process of doing it myself.

<img width="707" height="900" alt="image" src="https://github.com/user-attachments/assets/64dd6ae9-7ac8-4ad5-837d-ae60648bd1e8" />

#### Update 16/01/2026

After initially having issues getting the screen to turn on, I believe the issue was not initialising the device handler to zeros hence the charge pump bit was most likely set. So the screen was trying to use an external voltage which was not there. Anyhow.. I have tested the screen with making it all white! Very happy to see progress and I can almost taste the finishing line.

![image0 (4)](https://github.com/user-attachments/assets/bba30e50-9fd2-47a3-969a-25926fa0ea00)

#### Update 16/01/2026

Second test completed. This shows that the character to pixel mapping is working and the addressing mode (horizontal) is enabled correctly. 

![image0 (5)](https://github.com/user-attachments/assets/8e7b6b39-4ce1-41bf-bf24-6583553931e1)

#### Update 16/01/2026

## Project done. Very happy with the results and I have learnt a great deal by taking on this project.

![image0 (6)](https://github.com/user-attachments/assets/e944a601-eeb0-4851-8795-ac9ff983bfff)



