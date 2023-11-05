import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/serial0', baud=9600)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

the_connection.mav.request_data_stream_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)

while True:
    try:
       
        msg = the_connection.recv_match(blocking=True)
        if not msg:
            continue

        
        if msg.get_type() == 'RAW_ADC':
            print(f"ADC Channels: {msg.chan1_raw}, {msg.chan2_raw}, {msg.chan3_raw}, ...")
        elif msg.get_type() == 'ADC_REPORT':
            print(f"ADC Voltage: {msg.voltages}")

   
        time.sleep(0.5)
        
    except Exception as e:
        print(e)
        break
