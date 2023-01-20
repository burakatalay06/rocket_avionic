from ast import If
from operator import truediv
from pickle import FALSE, TRUE
import smbus
import time
import math

import logging
import sys

from Adafruit_BNO055 import BNO055

from threading import Thread

import time
from gpiozero import LED,Button
from PID import PID

#start=time.time()



#----cikis verilecek olan pinler----#
#GPIO27 ---> valve-1
#GPIO18 ---> valve-2
#GPIO22 ---> valve-3
#GPIO23 ---> valve-4
#GPIO24 ---> valve-5
#GPIO25 ---> valve-6
#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
#GPIO5 ---> motor_valve-1
#GPIO6 ---> motor_valve-2
#GPIO12 ---> motor_valve-3
#GPIO17 ---> misina 
#GPIO26 --->
#GPIO21 --->    
#------------------------------------




threadlist = []


is_check_sealevel = FALSE

bus = smbus.SMBus(1)


bno = BNO055.BNO055(serial_port='/dev/serial0')

# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
# bno = BNO055.BNO055(rst='P9_12')


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')



# valve'lerin frekansı 50ms, rasp 50nanosaniye
led = LED(17)

valve_1 = LED(27)
valve_2 = LED(18)
valve_3 = LED(22)
valve_4 = LED(23)
valve_5 = LED(24)
valve_6 = LED(25)

motor_valve_1 = LED(5)
motor_valve_2 = LED(6)
motor_valve_3 = LED(12)
#misina = LED(17)




def ms5611():
	global is_check_sealevel
# MS5611_01BXXX address, 0x77(119)
#		0x1E(30)	Reset command
	bus.write_byte(0x77, 0x1E)

	time.sleep(0.5)

	# Read 12 bytes of calibration data
	# Read pressure sensitivity
	data = bus.read_i2c_block_data(0x77, 0xA2, 2)
	C1 = data[0] * 256 + data[1]

	# Read pressure offset
	data = bus.read_i2c_block_data(0x77, 0xA4, 2)
	C2 = data[0] * 256 + data[1]

	# Read temperature coefficient of pressure sensitivity
	data = bus.read_i2c_block_data(0x77, 0xA6, 2)
	C3 = data[0] * 256 + data[1]

	# Read temperature coefficient of pressure offset
	data = bus.read_i2c_block_data(0x77, 0xA8, 2)
	C4 = data[0] * 256 + data[1]

	# Read reference temperature
	data = bus.read_i2c_block_data(0x77, 0xAA, 2)
	C5 = data[0] * 256 + data[1]

	# Read temperature coefficient of the temperature
	data = bus.read_i2c_block_data(0x77, 0xAC, 2)
	C6 = data[0] * 256 + data[1]

	# MS5611_01BXXX address, 0x77(118)
	#		0x40(64)	Pressure conversion(OSR = 256) command
	bus.write_byte(0x77, 0x40)

	time.sleep(0.5)

	# Read digital pressure value
	# Read data back from 0x00(0), 3 bytes
	# D1 MSB2, D1 MSB1, D1 LSB
	value = bus.read_i2c_block_data(0x77, 0x00, 3)
	D1 = value[0] * 65536 + value[1] * 256 + value[2]

	# MS5611_01BXXX address, 0x76(118)
	#		0x50(64)	Temperature conversion(OSR = 256) command
	bus.write_byte(0x77, 0x50)

	time.sleep(0.5)

	# Read digital temperature value
	# Read data back from 0x00(0), 3 bytes
	# D2 MSB2, D2 MSB1, D2 LSB
	value = bus.read_i2c_block_data(0x77, 0x00, 3)
	D2 = value[0] * 65536 + value[1] * 256 + value[2]

	dT = D2 - C5 * 256
	TEMP = 2000 + dT * C6 / 8388608
	OFF = C2 * 65536 + (C4 * dT) / 128
	SENS = C1 * 32768 + (C3 * dT ) / 256
	T2 = 0
	OFF2 = 0
	SENS2 = 0

	if TEMP >= 2000 :
		T2 = 0
		OFF2 = 0
		SENS2 = 0
	elif TEMP < 2000 :
		T2 = (dT * dT) / 2147483648
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4
		if TEMP < -1500 :
			OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500))
			SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2

	TEMP = TEMP - T2
	OFF = OFF - OFF2
	SENS = SENS - SENS2
	pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0) / 100.0
	cTemp = TEMP / 100.0
	fTemp = cTemp * 1.8 + 32

	if is_check_sealevel == FALSE:
		sealevel = pressure
		is_check_sealevel = TRUE

	altitude = 44330 * (1-(pressure/sealevel)**(1/5.255))


	# Output data to screen
	print ("Pressure : %.2f mbar" %pressure)
	print ("Temperature in Celsius : %.2f C" %cTemp)
	print ("Temperature in Fahrenheit : %.2f F" %fTemp)
	print ("Altitude : %.2f m" %altitude)

heading, roll, pitch=0,0,0

def BNO():
	global heading,roll,pitch
	while True:
		heading, roll, pitch = bno.read_euler()
		# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
		sys, gyro, accel, mag = bno.get_calibration_status()
		# Print everything out.
		print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
				heading, roll, pitch, sys, gyro, accel, mag))
		# Other values you can optionally read:
		# Orientation as a quaternion:
		#x,y,z,w = bno.read_quaterion()
		# Sensor temperature in degrees Celsius:
		#temp_c = bno.read_temp()
		# Magnetometer data (in micro-Teslas):
		#x,y,z = bno.read_magnetometer()
		# Gyroscope data (in degrees per second):
		#x,y,z = bno.read_gyroscope()
		# Accelerometer data (in meters per second squared):
		#x,y,z = bno.read_accelerometer()
		# Linear acceleration data (i.e. acceleration from movement, not gravity--
		# returned in meters per second squared):
		#x,y,z = bno.read_linear_acceleration()
		# Gravity acceleration data (i.e. acceleration just from gravity--returned
		# in meters per second squared):
		#x,y,z = bno.read_gravity()
		# Sleep for a second until the next reading.
		time.sleep(0.01)

pid=PID()

def valves_pitch():
	global pitch
	target=0
	while True:
		pid.SetPoint=target
		out=pid.update(feedback_value=pitch)
		print(out)
		if out < 0:
			valve_1.on()
			time.sleep(abs(out)/1000)
			print(f"pitch ekseni valve1 ms {abs(out)/1000}")
			valve_1.off()
		elif out > 0:
			valve_2.on()
			time.sleep(abs(out)/1000)
			print(f"pitch ekseni valve2 ms {abs(out)/1000}")

			valve_2.off()
		uyku=1-abs(out)/1000
		time.sleep(abs(uyku))


def valves_roll():
	global roll
	target=0
	while True:
		pid.SetPoint=target
		out=pid.update(feedback_value=roll)
		print(out)
		if out < 0:
			valve_3.on()
			time.sleep(abs(out)/1000)
			print(f"roll ekseni valve3 ms {abs(out)/1000}")
			valve_3.off()
		elif out > 0:
			valve_4.on()
			time.sleep(abs(out)/1000)
			print(f"roll ekseni valve4 ms {abs(out)/1000}")
			valve_4.off()

		uyku=1-abs(out)/1000
		time.sleep(abs(uyku))


def valves_yaw():
	global heading
	target=0
	while True:
		pid.SetPoint=target
		out=pid.update(feedback_value=heading)
		print(out)
		if out < 0:
			valve_5.on()
			time.sleep(abs(out)/1000)
			print(f"yaw ekseni valve5 ms {abs(out)/1000}")
			valve_5.off()
		elif out > 0:
			valve_6.on()
			time.sleep(abs(out)/1000)
			print(f"yaw ekseni valve6 ms {abs(out)/1000}")
			valve_6.off()

		uyku=1-abs(out)/1000
		time.sleep(abs(uyku))


		
start=time.time()

Thread(target=BNO).start()
Thread(target=valves_pitch).start()
Thread(target=valves_roll).start()
Thread(target=valves_yaw).start()
Thread(target=ms5611).start()

end=time.time()

print("süre",end-start)




