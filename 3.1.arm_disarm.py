import time
import pymavlink.mavutil as utility     #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink --> mavutil.py 
import pymavlink.dialects.v20.all as dialect #/home/dronedev/.local/lib/python3.8/site-packages/pymavlink/dialects/v20  -->all.py

# 기체에 접속
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")
#vehicle = utility.mavlink_connection('/dev/ttyUSB0', 115200)	#Telemetry # cd /dev/  여기서 맞는 포트 확인가능하세요
#vehicle = utility.mavlink_connection('/dev/ttyACM0', 57600)	#usb # cd /dev/ 여기서 자기컴에 맞는 포트 확인가능하세요
# 접속 신호 시간
vehicle.wait_heartbeat()

# 접속 기체의 SYSID SYSID>0 면 성공.
print("Connected to the vehicle")
print("Target system:", vehicle.target_system, "Target component:", vehicle.target_component)

while 1:
    
    VEHICLE_ARM = 1 # arm
      # vehicle arm message 클래스로 시동걸기
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

    time.sleep(5)   
    
     # vehicle disarm message function으로 시동 끄기
    vehicle.mav.command_long_send(vehicle.target_system,
                                vehicle.target_component,
                                utility.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0,
                                0)
    msg=vehicle.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(5)



