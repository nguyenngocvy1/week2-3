from pymavlink import mavutil
from threading import Thread
import threading
from geographiclib.geodesic import Geodesic
import sys
import time
from math import *

def connect(connection_str):
        # Start a connection listening to a UDP port
        the_connection = mavutil.mavlink_connection(connection_str)
        # Wait for the first heartbeat
        #   This sets the system and component ID of remote system for the link
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
        # Once connected, use 'the_connection' to get and send messages
        return the_connection

def mess(the_connection, name):
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(name,msg)

# def get_gps(the_connection):
#         try:
#                 gps = the_connection.location(relative_alt = True)
#                 return gps, gps.lat, gps.lng, gps.alt
#         except:
#                 print('no gps')
#                 gps = -1
#                 return gps, gps, gps, gps

# def auto_gps(the_connection):
#         try:
#                 f = open('week2-3\\gps.txt','a+')
#         except:
#                 f = open('week2-3\\gps.txt','w+')
#         count = 0
#         time_wait = 60 #1min = 60 second
#         print('start get gps to file gps.txt')
#         time_out = time.time() + time_wait
#         while True:
#                 if time.time() > time_out:
#                         break
#                 gps, _, _, _ = get_gps(the_connection)
#                 count += 1
#                 f.write(str(gps)+' ')
#         time.sleep(0.1)
#         f.write('\n')
#         f.close()
#         gps_per_second = count / time_wait
#         print('----- getting gps is finish -----')
#         print(gps_per_second,'gps/second')

# def set_home(the_connection):
#     the_connection.mav.command_long_send(
#         the_connection.target_system,
#         the_connection.target_component,    
#         mavutil.mavlink.MAV_CMD_DO_SET_HOME,
#         0,
#         1, 0, 0, 0, 0, 0, 0) # para 1: use current location
#     time.sleep(3)
#     mess(the_connection,'set home')
#     gps, _, _, _ = get_gps(the_connection)
#     print('home gps:',gps)

def set_mode(the_connection, mode):
        if mode not in the_connection.mode_mapping():
                print('Unknown mode : {}'.format(mode))
                print('Try:', list(the_connection.mode_mapping().keys()))
                sys.exit(1)
        # Get mode ID
        mode_id = the_connection.mode_mapping()[mode]
        # Set mode
        the_connection.mav.set_mode_send(the_connection.target_system,
                                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        mess(the_connection,'set mode')

def arm(the_connection):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                             1, 0, 0, 0, 0, 0, 0) #param1 (1: arm, 0: disarm)
        mess(the_connection,'arm')

def take_off(the_connection, alt):
# get home gps before take off
        # gps, _, _, _ = get_gps(the_connection)
        # print('home gps:',gps)
# --take off--
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                             0, 0, 0, 0, 0, 0, alt)
        mess(the_connection,'take off')
# -- waiting for taking off
        # while True:
        #         _, _, _, gps_alt = get_gps(the_connection)
        #         print(gps_alt)
        #         time.sleep(1)
        #         if gps_alt/alt >= 0.999:
        #                 print('----- taking off is finish -------')
        #                 break
        #         if gps_alt == -1:
        #                 break #error gps
        # time.sleep(1) #make sure that taking off is really finish
        time.sleep(30)

def move(the_connection, north, east, alt):
        down = -alt
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component,
                                                                                               mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000, #type_mask
                                                                                               north, east, down,
                                                                                               0, 0, 0, #v north, v east, v down
                                                                                               0, 0, 0, 
                                                                                               0, 0))
        mess(the_connection,'move')
#wait for move
        # while True:
        #         _, t1_gps_lat, t1_gps_lon, _ = get_gps(the_connection)
        #         print(t1_gps_lat)
        #         time.sleep(3)
        #         _, t2_gps_lat, t2_gps_lon, _ = get_gps(the_connection)
        #         print(t2_gps_lat)
        #         if (t2_gps_lat/t1_gps_lat >= 0.99 and t2_gps_lat/t1_gps_lat <= 1.01) and (t2_gps_lon/t1_gps_lon >= 0.99 and t2_gps_lon/t1_gps_lon <= 1.01):
        #                 print('----- You have reached your destination -----')
        #                 break
        # time.sleep(1)
        time.sleep(37)
        

# def move_to_gps(the_connection, destination_gps_lat, destination_gps_lon, alt):
#         geod = Geodesic.WGS84 # define the WGS84 ellipsoid
#         _, started_gps_lat, started_gps_lon, _ = get_gps(the_connection)
#         distance = geod.Inverse(started_gps_lat, started_gps_lon, destination_gps_lat, destination_gps_lon)
        
#         the_connection.mav.set_position_target_global_int_send(0, 
#                                                            the_connection.target_system,
#                                                            the_connection.target_component,
#                                                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#                                                            0b110111111000,
#                                                            int(destination_gps_lat*1e7), int(destination_gps_lon*1e7),alt,
#                                                            0,0,0,
#                                                            0,0,0,
#                                                            0,0)
       
def return_to_launch(the_connection):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
                                             0, 0, 0, 0, 0, 0, 0)
        mess(the_connection,'RTL')

def land(the_connection):
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                             mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                                             0, 0, 0, 0, 0, 0, 0)
        mess(the_connection,'land')
             
def main1():
        connection_string = 'tcp:localhost:5760'
        mode = 'GUIDED'
        altitude = 50 #meters
        move_north = 200
        move_east = 0
        the_connection = connect(connection_string)
        set_mode(the_connection, mode)
        # a = get_gps(the_connection,'alt')
        # print(a)
        arm(the_connection)
        take_off(the_connection, altitude)
        move(the_connection,move_north, move_east, altitude)
        # auto_gps(the_connection)
        try:
                return_to_launch(the_connection)
        except:
                move(the_connection, -move_north, -move_east, altitude)
                land(the_connection)
# def main2():
#         connection_string = 'tcp:localhost:5760'
#         mode = 'GUIDED'
#         altitude = 50 #meters
#         the_connection = connect(connection_string)
#         set_mode(the_connection, mode)
#         arm(the_connection)
#         take_off(the_connection, altitude)
#         auto_gps(the_connection)
#         return_to_launch(the_connection)

# def main3():
#         connection_string = 'tcp:localhost:5760'
#         mode = 'GUIDED'
#         altitude = 50 #meters
#         the_connection = connect(connection_string)
#         set_mode(the_connection, mode)
#         arm(the_connection)
#         take_off(the_connection, altitude)
#         move_to_gps(the_connection,5,6,altitude)
#         return_to_launch(the_connection)
        
if __name__ == "__main__":
        main1()
        # main2()
        # main3()