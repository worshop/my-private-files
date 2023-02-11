"""
    Source: https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT
            https://mavlink.io/en/messages/common.html#MAV_FRAME
            https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
            https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT
            https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
"""

import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import geopy.distance  #거리 측정 없으면 install -> pip install geopy

 
# 기체에 통신연결
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")

# 신호 대기
vehicle.wait_heartbeat()

# 기체 접속 상태
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# 현재 위치
current_location ={
    "latitude":0.0,
    "longitude":0.0
}

# 목표 좌표및 거리
# vehicle's target location
target_location = {
    "latitude": 0.0,
    "longitude": 0.0,
    "distance": 0.0
}

# loop
while True:
    message = vehicle.recv_match(type=[dialect.MAVLink_position_target_global_int_message.msgname,dialect.MAVLink_global_position_int_message.msgname],blocking=True)
    message = message.to_dict()
    # 현재 위성 좌표얻기
    if message["mavpackettype"] == dialect.MAVLink_global_position_int_message.msgname:
        current_location["latitude"] = message["lat"] * 1e-7
        current_location["longitude"] = message["lon"] * 1e-7

        # 현재 좌표 출력


    # 목표 좌표 얻기
    if message["mavpackettype"] == dialect.MAVLink_position_target_global_int_message.msgname:
        target_location["latitude"] = message["lat_int"] * 1e-7
        target_location["longitude"] = message["lon_int"] * 1e-7

        # 목표위치까지의 거리 계산
        distance = geopy.distance.GeodesicDistance((current_location["latitude"],
                                                    current_location["longitude"]),
                                                   (target_location["latitude"],
                                                    target_location["longitude"])).meters
        target_location["distance"] = distance
       
    print("Current",
    "Lat:", round(current_location["latitude"],4),
    "Lon:", round(current_location["longitude"],4) 
    , "Target",
    "Lat:", round(target_location["latitude"],4),
    "Lon:", round(target_location["longitude"],4),
    "Dis:", round(target_location["distance"],2) ,"m")