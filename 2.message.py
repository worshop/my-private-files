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



while True:

    # try to receive a message
    try:

        # receive a message /type = mavlink common lists https://mavlink.io/en/messages/common.html
        #message = vehicle.recv_match(type=dialect.MAVLink_system_time_message.msgname, blocking=True)
        message = vehicle.recv_match()        
        # convert received message to dictionary
        message = message.to_dict()
        print(message)
       
    except: 
        time.sleep(0.1)
