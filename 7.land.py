import time
import pymavlink.mavutil as utility #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink --> mavutil.py 
import pymavlink.dialects.v20.all as dialect #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink/dialects/v20  -->all.py

# connect to vehicle   mavutil.py  1079 line ->def mavlink_connection()
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560") #mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14560
#vehicle = utility.mavlink_connection('/dev/ttyUSB0', 57600)	#Telemetry # cd /dev/  여기서 맞는 포트 확인가능하세요
#vehicle = utility.mavlink_connection('/dev/ttyACM0', 57600)	#usb # cd /dev/ 여기서 자기컴에 맞는 포트 확인가능하세요

# wait for a heartbeat
vehicle.wait_heartbeat()  # mavutil.py 539 line -> def wait_hearbeat()

# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# infinite loop
# https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
vehicle_land_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=dialect.MAV_CMD_NAV_LAND,
    confirmation=0,
    param1=0,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)
vehicle.mav.send(vehicle_land_message)