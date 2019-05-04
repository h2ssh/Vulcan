import os
import configparser

os.system("ls /dev | grep pcan > pcan_available.txt")

config = configparser.ConfigParser()
config.sections()
config.read("build/bin/robot_interface.cfg")
#print (config.sections())
#print (config["WheelchairParameters"]["controller_port"])
#print (config["WheelchairParameters"]["joystick_port"])

file = open("pcan_available.txt", "r")
line_count = 0
for line in file:
	line_str = line.strip()
	line_str2 = "/dev/" + line_str
	#print(line_str2)
	if(line_count==0):
		config["WheelchairParameters"]["controller_port"] = line_str2
	elif(line_count==1):
		config["WheelchairParameters"]["joystick_port"] = line_str2
	else:
		break
	line_count= line_count+1

with open('build/bin/robot_interface.cfg', 'w') as configfile:
	config.write(configfile)
