import PID
import time
import os.path



targetT = 35
P = 10
I = 1
D = 1

pid = PID.PID(P, I, D)
pid.SetPoint = targetT
pid.setSampleTime(1)

def readConfig ():
	global targetT
	with open ('/tmp/pid.conf', 'r') as f:
		config = f.readline().split(',')
		pid.SetPoint = float(config[0])
		targetT = pid.SetPoint
		pid.setKp (float(config[1]))
		pid.setKi (float(config[2]))
		pid.setKd (float(config[3]))

def createConfig ():
	if not os.path.isfile('pid.conf'):
		with open ('pid.conf', 'w') as f:
			f.write('%s,%s,%s,%s'%(targetT,P,I,D))

createConfig()

while 1:
	#read temperature data
	temperature = (1 - 0.5) * 100

	pid.update(temperature)
	targetPwm = pid.output
	
	targetPwnabs=abs(targetPwm)
	targetPwm = max(min(int(targetPwm), 100),0)
	

	print("Target: %.1f C | Current: %.1f C | PWM: %s %%"%(targetT, temperature, targetPwm))

	# Set PWM expansion channel 0 to the target setting
	time.sleep(0.5)