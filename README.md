# STM32 Fingerprint RFID Door Lock

An STM32-based smart door lock system built with FreeRTOS.  
This project integrates fingerprint authentication, RFID cards, keypad password input, ultrasonic auto-open detection, servo motor control, and a 16x2 I2C LCD interface. It also supports local flash storage for enrolled fingerprint IDs and registered RFID cards.

## Features

- Fingerprint authentication with AS608
- RFID card authentication with RC522
- Keypad password login
- 16x2 I2C LCD user interface
- Servo-based door lock control
- Ultrasonic sensor auto-open mode
- Admin menu for adding and deleting RFID cards
- Admin menu for enrolling and deleting fingerprints
- Local flash storage for:
  - fingerprint ID map
  - registered RFID cards
- FreeRTOS task-based architecture

## Hardware

- STM32F103C8T6
- AS608 Fingerprint Sensor
- RC522 RFID Reader
- HC-SR04 Ultrasonic Sensor
- 16x2 I2C LCD
- 4x4 Keypad
- Servo Motor

## Pin Mapping

### LCD I2C
- I2C1 SCL: `PB6`
- I2C1 SDA: `PB7`

### Fingerprint Sensor (AS608)
- USART3 TX: `PB10`
- USART3 RX: `PB11`

### RFID Reader (RC522)
- SPI1 SCK: `PA5`
- SPI1 MISO: `PA6`
- SPI1 MOSI: `PA7`
- CS: `PA4`
- RST: `PB0`

### Ultrasonic Sensor (HC-SR04)
- TRIG: `PA1`
- ECHO: `PA0`

### Servo
- PWM: `PB1` (`TIM3_CH4`)

### Door Sensor
- Switch input: `PA2`

### Alarm LED
- LED: `PC14`

### Keypad
- Columns: `PA8`, `PB15`, `PB14`, `PB13`
- Rows: `PA9`, `PA10`, `PA11`, `PA12`

## Software Architecture

This project uses FreeRTOS and is organized into multiple tasks:

- **KeypadTask**: scans keypad input
- **RFIDTask**: reads RFID cards
- **FingerTask**: handles fingerprint scan, enroll, and delete
- **AutoTask**: monitors ultrasonic auto-open logic
- **LCDTask**: updates the LCD display
- **AccessTask**: processes system events and controls state transitions

## Main Functions

### Login Methods
- Admin password
- User password
- RFID card
- Fingerprint

### Admin Functions
- Add RFID card
- Delete RFID card
- Enroll fingerprint
- Delete fingerprint
- Change admin password
- Change user password
- Enable auto mode

### Auto Mode
When auto mode is enabled, the door can open automatically if an object is detected within the configured ultrasonic distance threshold.

## Flash Storage

This project stores important access data in STM32 internal flash:

- Fingerprint ID allocation map
- Registered RFID card list

This allows saved data to remain available after reset or power loss.

## Default Passwords

- Admin password: `000`
- User password: `111`

## Project Structure

```text
Core/
  Inc/
  Src/
Drivers/
Middlewares/
Startup/
hientai.ioc
STM32F103C8TX_FLASH.ld
.project
.cproject
.mxproject
How to Open the Project
Open STM32CubeIDE
Select File > Open Projects from File System
Choose this project folder
Open the .ioc file if you want to review or modify peripheral configuration
How to Build and Flash
Connect your STM32 board
Open the project in STM32CubeIDE
Build the project
Flash it to the board using ST-LINK
Notes
This project is designed for STM32F103-based hardware.
Flash page addresses may need adjustment if your target MCU or linker layout is different.
The Debug/ folder is not required in the repository and should not be committed.
.launch files are optional and can be excluded from version control.
Future Improvements
Save user settings in flash
Add EEPROM or external storage support
Improve LCD menu UX
Add buzzer or event logs
Add master reset or backup/restore featur
