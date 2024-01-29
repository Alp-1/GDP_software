import time
from pymavlink import mavutil

# Connect to the vehicle
the_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=9600)
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


def set_position_target_local_ned(x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, coordinate_frame, type_mask):
    the_connection.mav.set_position_target_local_ned_send(
        time_boot_ms=0,  
        target_system=the_connection.target_system,
        target_component=the_connection.target_component,
        coordinate_frame=coordinate_frame,
        type_mask=type_mask,
        x=x, y=y, z=z,
        vx=vx, vy=vy, vz=vz,
        afx=afx, afy=afy, afz=afz,
        yaw=yaw,
        yaw_rate=yaw_rate
    )



# Main loop
try:
    while True:
        # Other logic here
        time.sleep(1)

except Exception as e:
    print(e)

finally:
    the_connection.close()
