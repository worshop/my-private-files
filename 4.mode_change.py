#https://ardupilot.org/dev/docs/mavlink-commands.html
import time
import pymavlink.mavutil as utility     #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink --> mavutil.py 
import pymavlink.dialects.v20.all as dialect #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink/dialects/v20  -->all.py

# 기체에 접속
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")
#vehicle = utility.mavlink_connection('/dev/ttyUSB0', 57600)	#Telemetry # cd /dev/  여기서 맞는 포트 확인가능하세요
#vehicle = utility.mavlink_connection('/dev/ttyACM0', 57600)	#usb # cd /dev/ 여기서 자기컴에 맞는 포트 확인가능하세요

# 접속 신호 시간
vehicle.wait_heartbeat()

# 접속 기체의 SYSID SYSID>0 면 성공.
print("Connected to the vehicle")
print("Target system:", vehicle.target_system, "Target component:", vehicle.target_component)

# https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html

flight_modes = vehicle.mode_mapping()   
T_DO_COMMAND = dialect.MAV_CMD_DO_SET_MODE #https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE
T_MODE_ENABLED_PARAM1 = dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED #   value =1 same    https://mavlink.io/en/messages/common.html#MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
T_FLIGHT_MODE_PARAM2 = "GUIDED" # arm  https://ardupilot.org/dev/docs/mavlink-get-set-flightmode.html  mode-> number
  # vehicle arm message 클래스로 시동걸기
vehicle_arm_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=T_DO_COMMAND,
    confirmation=0,
    param1=T_MODE_ENABLED_PARAM1,
    param2=flight_modes[T_FLIGHT_MODE_PARAM2],
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)
vehicle.mav.send(vehicle_arm_message)

print(flight_modes[T_FLIGHT_MODE_PARAM2], T_FLIGHT_MODE_PARAM2)  # 
time.sleep(5)   
    
     

