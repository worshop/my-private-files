import pymavlink.mavutil
import time
# 기체에 접속
vehicle = pymavlink.mavutil.mavlink_connection(device="udpin:127.0.0.1:14560")
#vehicle = pymavlink.mavutil.mavlink_connection('/dev/ttyUSB0', 115200)	#Telemetry # cd /dev/  여기서 맞는 포트 확인가능하세요
#vehicle = utility.mavlink_connection('/dev/ttyACM0', 57600)	#usb # cd /dev/ 여기서 자기컴에 맞는 포트 확인가능하세요
# 접속 신호 시간
vehicle.wait_heartbeat(timeout=5)

# 접속 기체의 SYSID SYSID>0 면 성공.
print("Connected to the vehicle")
print("Target system:", vehicle.target_system, "Target component:", vehicle.target_component)

# target_system -> vehicle system ID target_component-> flight controller Element ID
 # >param set SYSID_THISMAV {target_system} 
