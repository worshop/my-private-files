import time
import tkinter as tk
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

# connect to vehicle
vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14560") 
# wait for a heartbeat
vehicle.wait_heartbeat()

def mavlinkStr1(cmdV, p1, p2, p3, p4, p5, p6, p7):

    vehicle_arm_message = dialect.MAVLink_command_long_message(
        target_system=vehicle.target_system,
        target_component=vehicle.target_component,
        command=cmdV,
        confirmation=0,
        param1=p1,
        param2=p2,
        param3=p3,
        param4=p4,
        param5=p5,
        param6=p6,
        param7=p7
    )
    vehicle.mav.send(vehicle_arm_message)

# define some target locations
TARGET_LOCATIONS = [
    {
        "latitude": 36.5940,
        "longitude": 126.2939,
        "altitude": 40
    },
    {
        "latitude": 36.5945,
        "longitude": 126.2888,
        "altitude": 40
    }
]

# vehicle's current location
current_location = {
    "latitude": 0.0,
    "longitude": 0.0
}

# vehicle's target location
target_location = {
    "latitude": 0.0,
    "longitude": 0.0,
    "distance": 0.0
}

root = tk.Tk()
root.title("MAVLINK PANNEL")
root.geometry("600x200")


down_frame = tk.Frame(root, width=600, height=100)
down_frame.pack(padx=10, pady=10)



btn_stb = tk.Button(down_frame,text='STABILIZE', padx=50, pady=10,command=lambda: mavlinkStr1(176,1,0,0,0,0,0,0))
btn_stb.grid(column=0, row=0, padx=5, pady=5)
btn_guided = tk.Button(down_frame,text='GUIDED', padx=50, pady=10,command=lambda: mavlinkStr1(176,1,4,0,0,0,0,0))
btn_guided.grid(column=1, row=0, padx=5, pady=5)
btn_auto = tk.Button(down_frame,text='AUTO', padx=50, pady=10,command=lambda: mavlinkStr1(176,1,3,0,0,0,0,0))
btn_auto.grid(column=2, row=0, padx=5, pady=5)

btn_arm = tk.Button(down_frame,text='Arming', padx=15, pady=10,command=lambda: mavlinkStr1(400,1,0,0,0,0,0,0))
btn_arm.grid(column=0, row=1, padx=5, pady=5)
btn_disarm = tk.Button(down_frame,text='Disarming', padx=15, pady=10,command=lambda: mavlinkStr1(400,0,0,0,0,0,0,0))
btn_disarm.grid(column=1, row=1, padx=5, pady=5)
btn_land = tk.Button(down_frame,text='Landing', padx=15, pady=10,command=lambda: mavlinkStr1(21,0,0,0,0,0,0,0))
btn_land.grid(column=2, row=1, padx=5, pady=5)
btn_rtl = tk.Button(down_frame,text='RTL', padx=15, pady=10,command=lambda: mavlinkStr1(20,0,0,0,0,0,0,0))
btn_rtl.grid(column=3, row=1, padx=5, pady=5)

 
 
btn_takeoff20 = tk.Button(down_frame,text='Takeoff20', padx=15, pady=10,command=lambda: mavlinkStr1(22,0,0,0,0,0,0,20))
btn_takeoff20.grid(column=1, row=2, padx=5, pady=5)

root.mainloop()
 

 
