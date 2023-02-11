import time

import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
#import geopy.distance

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")#시물레이터는 이거구요
#vehicle = utility.mavlink_connection('/dev/ttyUSB0', 57600)	#Telemetry # cd /dev/  여기서 맞는 포트 확인가능하세요
#vehicle = utility.mavlink_connection('/dev/ttyACM0', 57600)	#usb # cd /dev/ 여기서 자기컴에 맞는 포트 확인가능하세요

# wait for a heartbeat
vehicle.wait_heartbeat()

# desired flight mode
FLIGHT_MODE = "GUIDED"
# get supported flight modes
flight_modes = vehicle.mode_mapping()

# check the desired flight mode is supported
if FLIGHT_MODE not in flight_modes.keys():
	# inform user that desired flight mode is not supported by the vehicle
	print(FLIGHT_MODE, "is not supported")

	# exit the code
	exit(1)

# create change mode message
set_mode_message = dialect.MAVLink_command_long_message(
	target_system=vehicle.target_system,
	target_component=vehicle.target_component,
	command=dialect.MAV_CMD_DO_SET_MODE,
	confirmation=0,
	param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
	param2=flight_modes[FLIGHT_MODE],
	param3=0,
	param4=0,
	param5=0,
	param6=0,
	param7=0
)
vehicle.mav.send(set_mode_message)
time.sleep(2)

VEHICLE_ARM = 1
VEHICLE_DISARM = 0

# connect to vehicle
# vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")

# vehicle arm message
vehicle_arm_message = dialect.MAVLink_command_long_message(
	target_system=vehicle.target_system,
	target_component=vehicle.target_component,
	command=dialect.MAV_CMD_COMPONENT_ARM_DISARM,
	confirmation=0,
	param1=VEHICLE_ARM,
	param2=0,
	param3=0,
	param4=0,
	param5=0,
	param6=0,
	param7=0
)

vehicle.mav.send(vehicle_arm_message)
time.sleep(2)

# takeoff altitude definition
TAKEOFF_ALTITUDE = 50

# create takeoff command
takeoff_command = dialect.MAVLink_command_long_message(
	target_system=vehicle.target_system,
	target_component=vehicle.target_component,
	command=dialect.MAV_CMD_NAV_TAKEOFF,
	confirmation=0,
	param1=0,
	param2=0,
	param3=0,
	param4=0,
	param5=0,
	param6=0,
	param7=TAKEOFF_ALTITUDE
)

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# takeoff the vehicle
vehicle.mav.send(takeoff_command)     
