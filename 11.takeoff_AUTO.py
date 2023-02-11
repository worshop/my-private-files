"""
    Source: https://mavlink.io/en/messages/common.html#COMMAND_LONG
            https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF
            https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
            https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT
"""

import time
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560")
# wait for a heartbeat
vehicle.wait_heartbeat()
# inform user
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)


flight_modes = vehicle.mode_mapping()   
T_DO_COMMAND = dialect.MAV_CMD_DO_SET_MODE 
T_MODE_ENABLED_PARAM1 = dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 


def mode_chg(modev):
    vehicle_arm_message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=T_DO_COMMAND,
        confirmation=0,
        param1=T_MODE_ENABLED_PARAM1,
        param2=flight_modes[modev],
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0
    )
    return vehicle_arm_message 
    

VEHICLE_ARM = 1 # arm
  # vehicle arm message 클래스로 시동걸기
  #https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html
vehicle_arm_message = dialect.MAVLink_command_long_message(
    target_system=vehicle.target_system,
    target_component=vehicle.target_component,
    command=400,
    confirmation=0,
    param1=VEHICLE_ARM,
    param2=0,
    param3=0,
    param4=0,
    param5=0,
    param6=0,
    param7=0
)



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

 
 

#mode change
while True:
    vehicle.mav.send(mode_chg("GUIDED"))

    time.sleep(5)   
    break

# arming
while True:
    vehicle.mav.send(vehicle_arm_message)
    break

# takeoff
while True:
    # takeoff the vehicle
    vehicle.mav.send(takeoff_command)
    # catch GLOBAL_POSITION_INT message
    message = vehicle.recv_match(type=dialect.MAVLink_global_position_int_message.msgname,
                                 blocking=True)
    # convert message to dictionary
    message = message.to_dict()
    # get relative altitude
    relative_altitude = message["relative_alt"] * 1e-3
    # print out the message
    print("Relative Altitude", relative_altitude, "meters")
    # check if reached the target altitude
    if TAKEOFF_ALTITUDE - relative_altitude < 1:
        # print out that takeoff is successful
        print("Takeoff to", TAKEOFF_ALTITUDE, "meters is successful")
        # break the loop
        break

# mode auto
while True:
    vehicle.mav.send(mode_chg("AUTO"))
    break

 