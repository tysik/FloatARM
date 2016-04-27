# Float ARM

This is the low level controller for the Float Hovercraft project. The controller is based on ATSAM3X8E microcontroller (Arduino Due). It's purpose is to receive data from on-board computer via UART and produce appropriate PWM signals for the motors ESC's. 

## Data transfer 

The serial connection has the following settings: 

* Baud rate: 115 200 b/s
* Data bits: 8
* Parity: none
* Stop bits: 1

The frame transmitted from PC to the low level controller has the following structure:

```
struct Frame {
  uint16_t header;
  float left_motor;
  float right_motor;
  float center_motor;
  uint16_t crc;
}
```

where the header has a constant form of 0xAABB. The left_ and right_motor variables keep the values of pushing forces in newtons. The center_motor variable keeps the value of middle motor power in percents. After receiving full frame and checking the CRC the low level controller sends an acknowledge information in form of a one byte 0xAC.

## Configuring Atmel Studio 6.1

The project is configured for Atmel Studio 6.1. In order to bootload and flash Arduino Due from Atmel Studio 6.1 the Arduino IDE is still needed. One must create a batch file called `ArduinoDueProgrammer.bat` with the following body:

```
mode %1:1200,n,8,1,p
 
<Arduino Installation Path>\hardware\tools\bossac.exe --port=%1 -U false -e -w -v
```

where the `<Arduino Installation Path>` must be changed to point to the directory where Arduino software was installed. It is worth creating the batch file in the same folder where the `bossac.exe` file exist in Arduino folder. Next, in Atmel Studio, one must create an external tool (`Tools -> External Tools...`) which will call the script. The input fields must be filled as follows:

* Title: Arduino Due Programmer
* Command: C:\Program Files (x86)\Arduino\hardware\tools\DueProgrammer.bat
* Arguments: COM15 $(TargetDir)$(TargetName).bin
* Initial Directory: $(TargetDir)

and the ticks should be:

* Use Output Window (Yes)
* Prompt for arguments (No)
* Treat output as Unicode (No)

The COM port number may obviously differ, so in that case it must be changed accordingly. The same apply to the path of the batch file. This way, the user can build the binary files and flash them to the ATSAM3X8E of Arduino Due using `Tools -> Arduino Due programmer`.