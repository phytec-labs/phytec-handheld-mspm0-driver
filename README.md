## Peripherals & Pin Assignments

| MSPM0L1306 (U9) Pin | Usage |	Peripheral |
| --- | --- | --- |
| PA27/A0 (pin 2) |	Button |	Right Angle Button |
| PA26/A1 (pin 1) |	Button |	Right Angle Button1 |
| PA25/A2 (pin 28) |	Joystick |	X2 (AD1) (pin 1) |
| PA24/A3 (pin 27) |	Joystick |	X2 (AD2) (pin 4) |
| PA22/A4 (pin 25) |	Joystick |	X3 (AD1) (pin 1) |
| PA21/A5/VREF+ (pin 24) |	Joystick |	X3 (AD2) (pin 4) |
| PA18/A7 |	MSPM0_BSL_INVOKE | |
| PA16/A8 |	Joy2_Click |	X3 (Switch) (pin 3) |
| PA15/A9 |	Joy1_Click |	X2 (Switch) (pin 3) |
| PA0 (pin 4) |	MSPM0_SDA |	|
| PA1 (pin 5) |	MSPM0_SCL |	|
| PA2/ROSC (pin 9)	Button	Button |
| PA3 (pin 10) |	Button |	Button1 |
| PA4 (pin 11) |	Button |	Button2 |
| PA5/HS (pin 12) |	Button	Button3 |
| PA6 (pin 13) |	Button |	Button4 |
| PA9 (pin 14) |	Button |	Button5 |
| PA10/HS (pin 15) |	Button |	Button6 |
| PA11 (pin 16) |	Button |	Button7 |
| PA14 (pin 17) |	Button |	Button8 |
| PA17 (pin 20) |	Button |	Button9 |
| PA23/VREF+ (pin 26) |	Button |	Button10 |

## Draft I2C Register Map

| Register | Descrption | Remark |
| --- | --- | --- |
| 0x1 | FW version | Major.Minor |
| 0x2 | SW Reset | |
| 0x3 | Button State | |
| 0x4 | ADC Values | |
