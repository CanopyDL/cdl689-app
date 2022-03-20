#Using the Application

##ModBus Interface

For clarity, lets use these acronyms:

- MWR(register, value1) means modbus write register with value1

- MRR(register) means modbus read register

The microcontroller acts as a bridge between modbus and the MEMS IMU.  Modbus has its own registers, coils, inputs, etc. that are separate from the MEMS device.  These Modbus registers have specific functions that enable control of the MEMS device over RS-485.

- Modbus register #1 is a bridge to read from the IMU

- Modbus register #2 is a bridge to write to the IMU

- Modbus register #3 is a data scratchpad

- Modbus register #10 is a switch for turning on and off streaming mode

- Modbus register #11 is a parameter for controlling the streaming update rate.  Write the update rate as an integer in microseconds to this register.

To read a register on the IMU, the host application needs to perform two steps:

MWR(1, x) where X is the register on the IMU that it wants to read

MRR(3)

To write a register on the IMU, the host application needs to perform two steps:

MWR(3, value)

MWR(2, x) where x is the register to write to

For example, the looking at the data sheet for the LSM6DSR sensor, the accelerometers start up in “power down mode.” They are controlled by register 0x10.

So, to turn on the accelerometers, we need to write 0x50 to LSM6DSR register 0x10.

MWR(3, 0x50)

MWR(2, 0x10)

##Streaming

Streaming is turned on by writing 0x69 to modbus register 10

Streaming is turned off by writing 0x96 to modbus register 10

Once streaming is turned on, the device starts and periodically outputs 12 bytes of data followed by an EOL sequence.

Gyro X low

Gyro X high

Gyro Y low

Gyro Y high

Gyro Z low

Gyro Z high

Accel X low

Accel X high

Accel Y low

Accel Y high

Accel Z low

Accel Z high

0x55

0xAA
