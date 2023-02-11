import pymavlink.mavutil
import time

# 기체에 접속
vehicle = pymavlink.mavutil.mavlink_connection(device="udpin:127.0.0.1:14560")
master = pymavlink.mavutil.mavlink_connection('udpout:localhost:14550', source_system=20)
master.mav.statustext_send(pymavlink.mavutil.mavlink.MAV_SEVERITY_NOTICE,"QGC will read this".encode())
                           