# Thermometer
Thermometer source code
This is an implementation of a device that counts the temperature using an NTC and an ATMega328p microprocessor. The device consists of the NTC, the microprocessor,
a heater that starts functioning while the temperature is less that 5oC and stops its operation when the temperature becomes more than 15oC. There is also a button,
when pressed a test-mode begins, while a red and a green led show at which state is the machine each time. The implementation of a timer for the functionalities of 
the device is pivotal. The program prints the data through the selected serial port.
Apart from the firmware, I have implemented a .NET GUI in order each operation to be easier for the user. Also, the latest implementation of the .NET application
stores everything the user does into a SQLite database.
