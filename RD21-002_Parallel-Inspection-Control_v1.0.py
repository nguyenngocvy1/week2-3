
# ***************************************************************************************************************************
# ***********************************************************************     INIT ARGUMENTS    *****************************                                    
# ***************************************************************************************************************************
import argparse
from threading import Thread
import threading

parser = argparse.ArgumentParser(description='Auto mission for powerline inspection program.')
parser.add_argument("-p", "--print_string", help=   "Arg order:                          \
                                                    0- waypoint file ,                   \
                                                    1st - number inspection pylon pos,   \
                                                    2nd - line points between pylons,    \
                                                    3rd - Inspection distance for drone, \
                                                    4th - Mission speed,                 \
                                                    5th - Mission heading,               \
                                                    6th -  Keep heading or not (1 or 0), \
                                                    7th - reverse = -1 or 1,             \
                                                    8th - mode = single or dual,         \
                                                    9th - follow_terrain = on or off",nargs='*')
args = parser.parse_args()
print(args)
print(args.print_string)

try:
    file_path = str(args.print_string[0])
except:
    file_path = 'test.waypoints'  # default value
try:
    pylon_point = int(args.print_string[1])
except:
    pylon_point = 3 # default value
try:
    line_point = int(args.print_string[2])
except:
    line_point = 3 # default value
try:
    inspection_distance = int(args.print_string[3])
except:
    inspection_distance = 10
try:
    mission_speed = int(args.print_string[4])
except:
    mission_speed = 8
try:
    heading = float(args.print_string[5])
except:
    heading = 270
try:
    keep_direction = int(args.print_string[6])
except:
    keep_direction = 0
try:
    reverse = int(args.print_string[7])
except:
    reverse = 1
try:
    inspection_mode = str(args.print_string[8])
except:
    inspection_mode = 'parallel'
try:
    follow_terrain_mode = str(args.print_string[9])
except:
    follow_terrain_mode = 'on'

print("--------------------------------------    MISSION SETTINGS     --------------------------------------------")
print("INSPECTION MISION: ",file_path)
print("INSPECTION LINE POINTS: ",line_point)
print("INSPECTION DISTANCE: ", inspection_distance)
print("MISSION SPEED: ",mission_speed)
print("MISSION HEADING: ",heading)
print("MISSION KEEP DIRECTION: ",keep_direction)
print("REVERSE: ",reverse)
print("INSPECTION MODE: ",inspection_mode)
print("FOLLOW TERRAIN: ",follow_terrain_mode)
print("-------------------------------    END OF MISSION SETTINGS     --------------------------------------------")

# ***************************************************************************************************************************
# ************************************************************     INIT VARS AND IMPORT LIBS    *****************************                                    
# ***************************************************************************************************************************
from pymavlink import mavutil
import sys
import math
import time 
from threading import Thread

turn_off = 0
lat = 0
lon = 0
alt = 0 
roll = 0 
pitch = 0
yaw = 0 
lock = threading.Lock()
total_distance = 0
avg_speed = 0
total_capture = 0
next_gimbal_target = 0
connection_string = "tcp:localhost:5763"
#connection_string = "tcp:192.168.0.210:20002"
safety_distance = inspection_distance * 0.5
current_mode = 'RTL'
start_thread = False
end_mission = False
curr_loc = (0,0,0)
curr_loc_heading = (0,0,0)
curr_loc_goto = (0,0,0)
curr_loc_checkpos = (0,0,0)
curr_loc_gimbal = (0,0,0)
missed_waypoints = 0
msg_reic_rate = 20
msg_update_rate = 19
total_photo = 0
min_interval = 100
max_interval = 0
missed_list = []

print("---------------------------------------------------------")
print("Connecting to VIAN on: %s" % (connection_string,))

# ***************************************************************************************************************************
# ************************************************************     MAVLINK and GPS FUNCTIONS    *****************************                                    
# *************************************************************************************************************************** 
#                                                                                                                          **
# This block content main functions of mavlink protocol                                                                    **                                                                                         
#   wait conn                                                                                                              **
#   process mavlink msg                                                                                                    **
#   add GPS to exif                                                                                                        **
#   take GPS                                                                                                               **
#                                                                                                                          **
# ***************************************************************************************************************************
# ***************************************************************************************************************************
def checkGimbal(): # kiểm tra kết nối giữa jetson vs gimbal (heartbeat)
    global turn_off, master
    while turn_off == 0:
        master.mav.heartbeat_send(2,0,0,0,0)
        time.sleep(1)
        # message = master.recv_msg()
        # if message is not None:
        #     GPS = message.get_type()
        #     # #print(GPS)
        #     # #if GPS == 'HEARTBEAT' and message.type == '2':
        #     # if GPS == 'HEARTBEAT':
        #     #     print("Component:",message.type)
        #     #     print(message)
        #     #     print("--------------")
        #     # if GPS == 'MOUNT_ORIENTATION':
        #     #     print("GIMBAL POS:",message)
        #     #     print("--------------")
        #     #if GPS == '
        #     time.sleep(1)
        # if message is None:
        #     pass

def takeGPS():
    global turn_off
    while turn_off ==0:
        try:
            msg = master.recv_match()
            processMsg(msg)
            time.sleep(0.01)
        except:
            pass
            #logging.debug("take GPS failed")

def processMsg(message):
    #global lat,lon,alt,direction

    if not message:
        #GPIO.output(26, GPIO.LOW)
        return False
    if message.get_type() == 'HEARTBEAT':
        return True
    if message.get_type() == 'GLOBAL_POSITION_INT':
        global lat 
        lat = message.lat / 1.0e7
        global lon
        lon = message.lon / 1.0e7
        global alt
        alt = message.relative_alt / 1000.0
        return True
    if message.get_type() == 'VFR_HUD':
        global heading
        heading = message.heading
        return True
    if message.get_type() == 'ATTITUDE':
        global yaw
        yaw = message.yaw
        global pitch
        pitch = message.pitch
        global roll
        roll = message.roll
        return True
    return False

def add_GPS_exif_tags(file_path):
    global lat,lon,alt,heading

    metadata = pyexiv2.ImageMetadata(file_path)
    metadata.read()

	# Exif.GPSInfo.GPSLatitudeRef
	# Exif.GPSInfo.GPSLatitude

	# Exif.GPSInfo.GPSLongitudeRef
	# Exif.GPSInfo.GPSLongitude

	# Exif.GPSInfo.GPSAltitudeRef
	# Exif.GPSInfo.GPSAltitude

	# Exif.GPSInfo.GPSImgDirectionRef
	# Exif.GPSInfo.GPSImgDirection

	# Latitude
    neg, d_int, m_int, secs = dd_to_dms(lat)
    d_frac, m_frac, s_frac = float_to_rational(d_int), float_to_rational(m_int), float_to_rational(secs)

    key = 'Exif.GPSInfo.GPSLatitude'
    value = [ d_frac, m_frac , s_frac]
    metadata[key] = pyexiv2.ExifTag(key, value)

    key = 'Exif.GPSInfo.GPSLatitudeRef'
    if neg:
        metadata[key] = pyexiv2.ExifTag(key, 'S')
    else:
        metadata[key] = pyexiv2.ExifTag(key, 'N')

    # Longitude
    neg, d_int, m_int, secs = dd_to_dms(lon)
    d_frac, m_frac, s_frac = float_to_rational(d_int), float_to_rational(m_int), float_to_rational(secs)

    key = 'Exif.GPSInfo.GPSLongitude'
    value = [ d_frac, m_frac , s_frac]
    metadata[key] = pyexiv2.ExifTag(key, value)

    key = 'Exif.GPSInfo.GPSLongitudeRef'
    if neg:
        metadata[key] = pyexiv2.ExifTag(key, 'W')
    else:
        metadata[key] = pyexiv2.ExifTag(key, 'E')

    # Altitude
    key = 'Exif.GPSInfo.GPSAltitude'
    value = float_to_rational(alt)
    metadata[key] = pyexiv2.ExifTag(key, value)

    # key = 'Exif.GPSInfo.GPSAltitudeRef'
    # metadata[key] = pyexiv2.ExifTag(key, 0)

    # ImgDirection
    key = 'Exif.GPSInfo.GPSImgDirection'
    value = float_to_rational(direction)
    metadata[key] = pyexiv2.ExifTag(key, value)

    key = 'Exif.GPSInfo.GPSImgDirectionRef'
    metadata[key] = pyexiv2.ExifTag(key, 'M')

    metadata.write()

def wait_conn():
    """
    Sends a ping to stabilish the communication and awaits for a response
    """
    msg = None
    while not msg:
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        #print(msg)
        time.sleep(0.05)

def takeMSL():
    alt = 99999.999
    msg = None
    while True:
        try:
            #master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, # request data stream
            #                                          4, 1) # rate = 4
            msg = master.recv_match()
            if msg is not None:
                #print(msg)
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    #lat = msg.lat / 1.0e7
                    #lon = msg.lon / 1.0e7
                    alt = msg.alt / 1000.0
                    break
            time.sleep(0.1)
        except:
            pass
    return alt

def takeAGL():
    alt = 99999.999
    msg = None
    while True:
        try:
            #master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, # request data stream
            #                                          4, 1) # rate = 4
            msg = master.recv_match()
            if msg is not None:
                #print(msg)
                if msg.get_type() == 'TERRAIN_REPORT':
                    terrain_alt = msg.terrain_height
                    alt = msg.current_height
                    break
        except:
            pass
    return alt, terrain_alt

# ***************************************************************************************************************************
# ***************************************************************************************************************************

master = None
while master is None:
    try:
        print("mavlink connecting")
        master = mavutil.mavlink_connection(connection_string)
        time.sleep(0.1)
    except KeyboardInterrupt:
        print("exit")
        sys.exit(0)
    except:
        pass
wait_conn()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_system))

#master.mav.heartbeat_send(type = 6, autopilot, base_mode, custom_mode, system_status, mavlink_version=3):

# **************************************************************************
# *******************************************************    FOR DEBUG    **
# ************************************************************************** 
# GPS = master.location(relative_alt= True) #relative to the home position
# print(GPS.lat)
# print(GPS.lng)
# print(GPS.alt)

# **************************************************************************
# **********************************************     CHECK CONNECTION     **                                      
# ************************************************************************** 
# This function check if the program can receive any message from the drone
def checkConnection():
    global msg_reic_rate
    print("-----------------------------------------------")
    print("CHECK CONNECTION PROCESS START")
    msg = None
    while not msg:
        master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, # request data stream replace position = all for all msg
                                                      int(msg_reic_rate), 1) # rate = 10, turn on = 1
        msg = master.recv_match()
        time.sleep(0.01)
    print(msg)
    i=0
    while i<3:
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        message = master.recv_msg()
        if message is not None:
            GPS = message.get_type()
            #print(GPS)
            if GPS == 'HEARTBEAT' and message.type == '2':
                print(message)
                print(message.type)
            if GPS == 'STATUSTEXT':
                print(message)
            if GPS == 'CAMERA_FEEDBACK':
                print(message)
            if GPS == 'GLOBAL_POSITION_INT':
                print(message)
                i+=1
            
        time.sleep(0.01)
    print("CHECK CONNECTION PROCESS SUCCESS")
    print("-------------------------------")
    return True

def setMsgInterval():  
    global msg_reic_rate
    master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, # request data stream all 
                                                       msg_reic_rate, 0) # rate = 4, turn on all
    master.mav.request_data_stream_send(0, 0, 33, # request data stream position
                                                      msg_reic_rate*5, 1) # rate = 100 hz
    master.mav.request_data_stream_send(0, 0, 136, # request data stream position
                                                      msg_reic_rate*5, 1) # rate = 100 hz


    master.mav.command_long_send(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, #command
        0, #confirmation
        33,    # param 1, 33 = global_pos_int
        0,          # param 2 - interval in micro second
        0,          # param 3 - 7 : 0
        0,
        0,0,0)
    # while True:
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()
    #     if ack_msg:
    #         # Print the ACK result !
    #         print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    #         break
    
    master.mav.command_long_send(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, #command
        0, #confirmation
        136,    # param 1, 33 = global_pos_int
        0,          # param 2 - interval in micro second
        0,          # param 3 - 7 : 0
        0,
        0,0,0)  
    # while True:
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()
    #     if ack_msg:
    #         # Print the ACK result !
    #         print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    #         break


# ***************************************************************************************************************************
# *********************************************************************     COPTER FUNCTIONS    *****************************                                    
# *************************************************************************************************************************** 
#                                                                                                                          **
# This block content main functions to control the VIAN in Guided mode:                                                    **                                                                                         
#   arm_and_takeoff                                                                                                        **
#                                                                                                                          **
# ***************************************************************************************************************************
# ***************************************************************************************************************************
import time
import math

"""
AltHold (quad disarmed): Base_Mode=81 , Custom_Mode=2
Loiter (quad disarmed): Base_Mode=89 , Custom_Mode=5
Auto (quad disarmed): Base_Mode=89 , Custom_Mode=3
RTL (quad disarmed): Base_Mode=89 , Custom_Mode=6
AltHold (quad armed): Base_Mode=209 , Custom_Mode=2
Loiter (quad armed): Base_Mode=217 , Custom_Mode=5
Auto (quad armed): Base_Mode=217 , Custom_Mode=3
RTL (quad armed): Base_Mode=217 , Custom_Mode=6
"""
def checkArmed():
    while True:
        if master.motors_armed:
            print("MOTOR ARMED")
            break
        time.sleep(0.05)

def takeOff(alt = 45):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 3, 0, 0 , 0, alt) # 1,2: ignore,3: ascend rate(m/s), 4 - yaw heading,5 6 7 : x,y,z local (m)

    while True:
        # Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    print("TAKE OFF") # AUTO SET HOME POSITION WHEN TAKE OFF

def landing(): # local landing
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
    0,
    0, 1, 2, 0, 0 , 0, 0) # 1: ignore, 2: offset (m), 3: descent rate (m/s), 4: yaw angle (rad), 5,6,7: x y z

    while True:
        # Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

def RTL(): #return home
    i=0
    while i<5:
        master.mav.command_long_send(
        master.target_system,
        master.target_component,    
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        i+=1

    # while True:
    #     # Wait for ACK command
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()

    #     # Print the ACK result !
    #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    #     break

def armAndTakeoff(aTargetAltitude = 15):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    global master,lock, curr_loc_goto

    # ------------------------------------------------------------ Copter should arm in GUIDED mode
    # mode = 'GUIDED'
    # if mode not in master.mode_mapping():
    #     print('Unknown mode : {}'.format(mode))
    #     print('Try:', list(master.mode_mapping().keys()))
    #     sys.exit(1)
    # print(master.mode_mapping())
    # mode_id = master.mode_mapping()[mode]

    # master.mav.set_mode_send(
    #     master.target_system,
    #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    #     mode_id)
    print ("SEND ARMING SIGNAL")
    i = 0
    while i < 5:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,    
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0) # para 1: 1 to ARM, 0 to DISARM
        i+=1
        # --------------------------------------------------------- Wait for ACK command
    # -------------------------------------------------------------- Confirm vehicle armed before attempting to take off
    checkArmed()
    
    print("Armed")

    print("Set new home location to current location")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,    
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1, 0, 0, 0, 0, 0, 0) # para 1: use current location

    print ("Taking off!")
    while turn_off == 0:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 3, 0, 0 , 0, aTargetAltitude) # 1,2: ignore,3: ascend rate(m/s), 4 - yaw heading,5 6 7 : x,y,z local (m)

        # while True:
        #     # Wait for ACK command
        #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        #     ack_msg = ack_msg.to_dict()

        #     # Print the ACK result !
        #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        #     break

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        time_start = time.time()
        while time.time()-time_start < 100 and turn_off == 0:
            while turn_off == 0:
                try:
                    currentLocation = curr_loc_goto
                    time.sleep(1/msg_update_rate)
                    break
                except:
                    pass
            temp_alt = currentLocation[2]
            if time.time() - time_start > 1:
                print (" Altitude: ", temp_alt)
                time_start = time.time()
            #Break and return from function just below target altitude.
            if temp_alt >= aTargetAltitude*0.95:
                break
            time.sleep(0.1)

        if temp_alt >= aTargetAltitude*0.95:
            print("Take of altitude reached: ",aTargetAltitude)
            break

def setSpeed(speed = 5):
    i=0
    while i < 5:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1, speed, -1, 0, 0 ,0, 0) # Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed), speed value, 0, relative: absolute - 0, relative 1, 5 6 7 : empty
        
        i+=1
    # while True:
    #     # Wait for ACK command
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()

    #     # Print the ACK result !
    #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    #     break
    print("Set ground speed to ",speed," m/s")

def setHeading(setHeading, relative= False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    global curr_loc_heading
    if master is None:
        print("PLease specific your vehicle to send yaw command")
    else:
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        if setHeading < 0 :
            direction = -1
        else:
            direction = 1

    while True:
        master.mav.command_long_send(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            round(setHeading,2),    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            direction,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        
        time_start = time.time()
        while time.time() - time_start < 3: #3sec time out
            #print(time.time() - time_start)
            while True:
                try:
                    currentLocation  = curr_loc_heading
                    time.sleep(1/msg_update_rate)
                    break
                except:
                    pass

            temp_heading = currentLocation[6]
            print("Changing heading to ",setHeading)
            print ("Heading ", temp_heading)
            #Break and return from function just below target altitude.
            if abs(temp_heading - setHeading) < 1: # 1 degree threshold
                break
            time.sleep(1)

        if abs(temp_heading - setHeading) < 1: # 1 degree error 
            print ("Reached target heading: ",setHeading)
            break
        else:
            print("resend heading")

def changeMode(mode='LOITER'):
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    mode_id = master.mode_mapping()[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    while True:
        # --------------------------------------------------------- Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # --------------------------------------------------------- Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue

        # --------------------------------------------------------- Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

def goTo(lat=10.8266951,lng=106.8252427,alt=30,heading=180):
    """
    Send move command to copter in global position and wait until the copter reachs target.
    """
    global total_distance, curr_loc_goto
    threshold = 1
    resend_count = 0
    print(" ")
    while True:
        try:
            currentLocation = curr_loc_goto
            time.sleep(1/msg_update_rate)
            break
        except:
            pass
    targetLocation = (lat,lng,alt)
    print("Going to target: ",targetLocation)

    temp_lat = currentLocation[0]
    temp_lng = currentLocation[1]
    temp_alt = currentLocation[4]

    print(" ---- ")
    print("Set heading to: ",heading)
    setHeading(heading, False)
    print(" ---- ")

    loc1 = (temp_lat,temp_lng,temp_alt)
    loc2= targetLocation
    targetDistance = getDistanceMetres(loc1, loc2)
    total_distance += targetDistance
    #print("check 1") # ----------------------------------------------------------------------- debug
    # mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    while True: # signal has to be sent every 1 second to avoid stuck
        # FOLLOW TERRAIN -------------------------------------------------------------------------------------------------------------------------------------            
        if follow_terrain_mode == 'on':
            master.mav.set_position_target_global_int_send(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                11, # frame MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11: relative to terrain, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6 : relative to home
                0b0000111111111000, # type_mask (only speeds enabled)
                int(lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
                int(lng*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
                round(float(alt),2), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
                0, # X velocity in NED frame in m/s
                0, # Y velocity in NED frame in m/s
                0, # Z velocity in NED frame in m/s
                0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
                round(heading,2), 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

            # master.mav.command_long_send( # spline waypoint trying - not success yet.
            #     master.target_system,
            #     master.target_component,
            #     mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT,
            #     0,
            #     0,0,0,0,lat,lng,round(float(alt),2))



            # while True:
            #     # Wait for ACK command
            #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
            #     ack_msg = ack_msg.to_dict()

            #     # Print the ACK result !
            #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            #     break

            #print("check 2") # ----------------------------------------------------------------------- debug

            time_start = time.time()
            #remainingDistance = getDistanceMetres(loc1, loc2)
            #remainingAltitude = (currentLocation.alt - alt)

            time_start = time.time()
            time_start_check = time.time()
            print("  ")
            time_start = time.time()
            while True:

                # currentLocation = master.location(relative_alt= True) # relative : use GLOBAL_POSOTION_INT
                # temp_lat = currentLocation.lat
                # temp_lng = currentLocation.lng
                # temp_alt = currentLocation.alt
                # MSL_alt = takeMSL()
                # AGL_alt,terrain_height = takeAGL()
                while True:
                    try:
                        currentLocation = curr_loc_goto
                        time.sleep(1/msg_update_rate)
                        break
                    except:
                        pass
                
                temp_lat = currentLocation[0]
                temp_lng = currentLocation[1]
                temp_alt = currentLocation[4]
                loc1 = (temp_lat,temp_lng,temp_alt)
                MSL_alt = currentLocation[3]
                AGL_alt = currentLocation[4]
                terrain_height = currentLocation[5]
                loc1 = (temp_lat,temp_lng,temp_alt)
                #print("check 3") # ----------------------------------------------------------------------- debug
                #print(loc1)
                #print(loc2)
                #if time.time()- time_start >1:
                    # print(" ")
                    # print("Relative home ALT:",round(temp_alt,3))
                    # print("MSL ALT: ",round(MSL_alt,3))
                    # print("AGL ALT: ",AGL_alt)
                    # print("Terrain height: ",terrain_height)
                    # print(" ")
                    #time_start = time.time()
                
                remainingDistance = round(getDistanceMetres(loc1, loc2),2)
                remainingAltitude = round((temp_alt - alt),2)

                if time.time() - time_start > 1: #update status every 1 sec
                    print(" ")
                    print("Goto <> Distance to target: ", remainingDistance," || Altitude to target: ", remainingAltitude)
                    print(" ")
                    time_start = time.time()

                if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 2.5*threshold  : #Just below target, in case of undershoot.
                    break
                else:
                    if time.time() - time_start_check > 5:  # static check time out 5sec
                        check = 0
                        prev_remainingDistance = remainingDistance
                        prev_remainingAltitude = remainingAltitude
                        print(" ")
                        print("Checking static")
                        while check < 10:

                            while True:
                                try:
                                    currentLocation = curr_loc_goto
                                    time.sleep(1/msg_update_rate)
                                    break
                                except:
                                    pass
                            
                            temp_lat = currentLocation[0]
                            temp_lng = currentLocation[1]
                            temp_alt = currentLocation[4]
                            loc1 = (temp_lat,temp_lng,temp_alt)
                            MSL_alt = currentLocation[3]
                            AGL_alt = currentLocation[4]
                            terrain_height = currentLocation[5]
                            loc1 = (temp_lat,temp_lng,temp_alt)

                            #if time.time() - time_start >1:
                                # print(" ")
                                # print("Relative home ALT:",round(temp_alt,3))
                                # print("MSL ALT: ",round(MSL_alt,3))
                                # print("AGL ALT: ",AGL_alt)
                                # print("Terrain height: ",terrain_height)
                                # print(" ")
                                #time_start = time.time()

                            loc1 = (temp_lat,temp_lng,temp_alt)
                            loc2= targetLocation

                            remainingDistance = round(getDistanceMetres(loc1, loc2),3)
                            remainingAltitude = round((temp_alt - alt),3)
                            #print(check," Distance to target: ", remainingDistance," |-| Altitude to target: ", remainingAltitude)
                            if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 2.5*threshold  : # If the drone reachs target, then disband the check
                                print("Target at ",round(targetDistance,3),"m away"," altitude ",round(alt,3),"m reached")
                                print(" ")
                                break

                            if (abs(remainingDistance - prev_remainingDistance) < 0.5 and abs(remainingAltitude - prev_remainingAltitude) < 0.5 \
                                and abs(remainingAltitude) < 0.5 and abs(remainingDistance) < 0.5) :
                                #print(check, remainingDistance - prev_remainingDistance," || ",remainingAltitude - prev_remainingAltitude)
                                check += 1
                            else:
                                print("Drone is moving")
                                time_start = time.time()
                                time_start_check = time.time()
                                break
                            time.sleep(1/msg_update_rate)
                        if check == 10:
                            break # break the loop and re send the command
                        else:
                            print("Reset static checking")
                            time_start = time.time() # reset the timer
                            time_start_check = time.time()
                            continue
                time.sleep(1/msg_update_rate)

            if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 2.5*threshold  : #Just below target, in case of undershoot.
                print("Target at ",round(targetDistance,3),"m away"," altitude ",round(alt,3),"m reached")
                print(" ")
                break
            else:   
                if resend_count < 2: # resend 3 times
                    print(" ")
                    print("Resend Goto() command: ",resend_count)
                    resend_count+=1
                else:
                    print(" ")
                    print("Repos activated")
                    resend = 0
                    rePos((targetLocation[0],targetLocation[1],targetLocation[2]),heading)

        # NO FOLLOW TERRAIN -------------------------------------------------------------------------------------------------------------------------------------            
        else:
            master.mav.set_position_target_global_int_send(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                6, # frame MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11: relative to terrain, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6 : relative to home
                0b0000111111111000, # type_mask (only speeds enabled)
                int(lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
                int(lng*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
                round(float(alt),2), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
                0, # X velocity in NED frame in m/s
                0, # Y velocity in NED frame in m/s
                0, # Z velocity in NED frame in m/s
                0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
                round(heading,2), 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

            # while True:
            #     # Wait for ACK command
            #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
            #     ack_msg = ack_msg.to_dict()

            #     # Print the ACK result !
            #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            #     break

            #print("check 2") # ----------------------------------------------------------------------- debug

            time_start = time.time()
            remainingDistance = getDistanceMetres(loc1, loc2)
            remainingAltitude = (currentLocation.alt - alt)

            time_start = time.time()
            time_start_check = time.time()
            print("  ")
            while True:

                currentLocation = master.location(relative_alt= True) # relative : use GLOBAL_POSOTION_INT
                temp_lat = currentLocation.lat
                temp_lng = currentLocation.lng
                temp_alt = currentLocation.alt
                MSL_alt = takeMSL()
                AGL_alt,terrain_height = takeAGL()

                loc1 = (temp_lat,temp_lng,temp_alt)
                curr_loc = (temp_lat,temp_lng,temp_alt) # update global var for photo capture thread
                #print("check 3") # ----------------------------------------------------------------------- debug
                #print(loc1)
                #print(loc2)
                
                remainingDistance = round(getDistanceMetres(loc1, loc2),2)
                print("*********")
                print("Relative home ALT:",round(temp_alt,3))
                print("*********")
                print("MSL ALT: ",round(MSL_alt,3))
                print("AGL ALT: ",AGL_alt)
                print("Terrain height: ",terrain_height)
                remainingAltitude = round((temp_alt - alt),2)


                if time.time() - time_start > 1: #update status every 1 sec
                    print("*********")
                    print("Distance to target: ", remainingDistance," || Altitude to target: ", remainingAltitude)
                    print("*********")
                    time_start = time.time()

                if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 2.5*threshold  : #Just below target, in case of undershoot.
                    break
                else:
                    if time.time() - time_start_check > 5:  # static check time out 5sec
                        check = 0
                        prev_remainingDistance = remainingDistance
                        prev_remainingAltitude = remainingAltitude
                        print(" ")
                        print("Checking static")
                        while check < 10:

                            currentLocation = master.location(relative_alt= True)
                            temp_lat = currentLocation.lat
                            temp_lng = currentLocation.lng
                            temp_alt = currentLocation.alt
                            MSL_alt = takeMSL()
                            AGL_alt,terrain_height = takeAGL()
                            curr_loc = (temp_lat,temp_lng,temp_alt) # update global var for photo capture thread
                            print("*********")
                            print("Relative home ALT:",round(temp_alt,3))
                            print("*********")
                            print("MSL ALT: ",round(MSL_alt,3))
                            print("AGL ALT: ",AGL_alt)
                            print("Terrain height: ",terrain_height)
                            #temp_alt = takeAltRelative()

                            loc1 = (temp_lat,temp_lng,temp_alt)
                            loc2= targetLocation
                            remainingDistance = round(getDistanceMetres(loc1, loc2),3)
                            remainingAltitude = round((temp_alt - alt),3)
                            #print(check," Distance to target: ", remainingDistance," |-| Altitude to target: ", remainingAltitude)
                            if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 2.5*threshold  : # If the drone reachs target, then disband the check
                                print("Target at ",round(targetDistance,3),"m away"," altitude ",round(alt,3),"m reached")
                                print(" ")
                                break

                            if (abs(remainingDistance - prev_remainingDistance) < 0.5 and abs(remainingAltitude - prev_remainingAltitude) < 0.5 \
                                and abs(remainingAltitude) < 0.5 and abs(remainingDistance) < 0.5) :
                                #print(check, remainingDistance - prev_remainingDistance," || ",remainingAltitude - prev_remainingAltitude)
                                check += 1
                            else:
                                print("Drone is moving")
                                time_start = time.time()
                                time_start_check = time.time()
                                break
                            time.sleep(0.2)
                        if check == 10:
                            break # break the loop and re send the command
                        else:
                            print("Reset static checking")
                            time_start = time.time() # reset the timer
                            time_start_check = time.time()
                            continue
                time.sleep(0.1)

            if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 2.5*threshold  : #Just below target, in case of undershoot.
                print("Target at ",round(targetDistance,3),"m away"," altitude ",round(alt,3),"m reached")
                print(" ")
                break
            else:   
                if resend_count < 2: # resend 3 times
                    print(" ")
                    print("Resend Goto() command: ",resend_count)
                    resend_count+=1
                else:
                    print(" ")
                    print("Repos activated")
                    resend = 0
                    rePos((targetLocation[0],targetLocation[1],targetLocation[2]),heading)

def goToLowAcc(lat=10.8266951,lng=106.8252427,alt=30,heading=180):
    """
    Send move command to copter in global position and wait until the copter reachs target.
    """
    global total_distance
    threshold = 0.3
    resend_count = 0
    print(" ")
    currentLocation = master.location(relative_alt= True)
    targetLocation = (lat,lng,alt)
    print("Going to target: ",targetLocation)
    temp_lat = currentLocation.lat
    temp_lng = currentLocation.lng
    temp_alt = currentLocation.alt
    print("Set heading to: ",heading)
    setHeading(heading, False)

    loc1 = (temp_lat,temp_lng,temp_alt)
    loc2= targetLocation
    targetDistance = getDistanceMetres(loc1, loc2)
    total_distance += targetDistance
    #print("check 1") # ----------------------------------------------------------------------- debug
    while True: # signal has to be sent every 1 second to avoid stuck
        master.mav.set_position_target_global_int_send(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            int(lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
            int(lng*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
            round(float(alt),2), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            round(heading,2), 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # while True:
        #     # Wait for ACK command
        #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        #     ack_msg = ack_msg.to_dict()

        #     # Print the ACK result !
        #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        #     break

        #print("check 2") # ----------------------------------------------------------------------- debug

        time_start = time.time()
        remainingDistance = getDistanceMetres(loc1, loc2)
        remainingAltitude = (currentLocation.alt - temp_alt)

        time_start = time.time()
        time_start_check = time.time()
        print("  ")
        while True:
            currentLocation = master.location(relative_alt= True)
            temp_lat = currentLocation.lat
            temp_lng = currentLocation.lng
            temp_alt = currentLocation.alt
            loc1 = (temp_lat,temp_lng,temp_alt)
            #print("check 3") # ----------------------------------------------------------------------- debug
            #print(loc1)
            #print(loc2)
            
            remainingDistance = round(getDistanceMetres(loc1, loc2),2)
            remainingAltitude = round((temp_alt - targetLocation[2]),2)


            if time.time() - time_start > 1: #update status every 1 sec
                print("Distance to target: ", remainingDistance," || Altitude to target: ", remainingAltitude)
                time_start = time.time()

            if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< threshold  : #Just below target, in case of undershoot.
                break
            else:
                if time.time() - time_start_check > 5:  # static check time out 5sec
                    check = 0
                    prev_remainingDistance = remainingDistance
                    prev_remainingAltitude = remainingAltitude
                    print("Checking static")
                    while check < 10:
                        temp_lat = currentLocation.lat
                        temp_lng = currentLocation.lng
                        temp_alt = currentLocation.alt
                        loc1 = (temp_lat,temp_lng,temp_alt)
                        loc2= targetLocation
                        remainingDistance = round(getDistanceMetres(loc1, loc2),3)
                        remainingAltitude = round((currentLocation.alt - targetLocation[2]),3)
                        print(check," Distance to target: ", remainingDistance," |-| Altitude to target: ", remainingAltitude)
                        if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< threshold  : # If the drone reachs target, then disband the check
                            print("Target at ",round(targetDistance,3),"m away"," altitude ",round(targetLocation[2],3),"m reached")
                            print(" ")
                            break

                        if (abs(remainingDistance - prev_remainingDistance) < 0.1 and abs(remainingAltitude - prev_remainingAltitude) < 0.1 \
                            and abs(remainingAltitude) < 0.5 and abs(remainingDistance) < 0.5) :
                            print(check, remainingDistance - prev_remainingDistance," || ",remainingAltitude - prev_remainingAltitude)
                            check += 1
                        else:
                            print("Drone is moving")
                            time_start = time.time()
                            time_start_check = time.time()
                            break
                        time.sleep(0.2)
                    if check == 10:
                        break # break the loop and re send the command
                    else:
                        print("Reset static checking")
                        time_start = time.time() # reset the timer
                        time_start_check = time.time()
                        continue
            time.sleep(0.1)

        if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< threshold  : #Just below target, in case of undershoot.
            print("Target at ",round(targetDistance,3),"m away"," altitude ",round(targetLocation[2],3),"m reached")
            print(" ")
            break
        else:   
            if resend_count < 2: # resend 3 times
                print("Resend Goto() command: ",resend_count)
                resend_count+=1
            else:
                print("Repos activated")
                resend = 0
                rePos((targetLocation[0],targetLocation[1],targetLocation[2]),heading)

def gotoPositionTargetGlobalInt(Location):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        Location[0]*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        Location[1]*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        Location[2], # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
    # #print "DEBUG: mode: %s" % vehicle.mode.name
    # remainingDistance=getDistanceMetres(vehicle.location.global_relative_frame, targetLocation)
    # print("Distance to target: ", remainingDistance)
    # if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
    #     print("Reached target")
    #     break

def gotoPositionTargetLocalNed( north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
    # #print "DEBUG: mode: %s" % vehicle.mode.name
    # remainingDistance=getDistanceMetres(vehicle.location.global_relative_frame, targetLocation)
    # print("Distance to target: ", remainingDistance)
    # if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
    #     print("Reached target")
    #     break

def getLocationMetres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def getDistanceMetres(loc1, loc2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    deltaLat = loc2[0] - loc1[0]
    deltaLong = loc2[1] - loc1[1]
    #print(deltaLat,deltaLong)
    return math.sqrt((deltaLat*deltaLat) + (deltaLong*deltaLong)) * 1.113195e5

def getBearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.
    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """	
    # off_x = float(aLocation2[0]) - float(aLocation1[0])
    # off_y = float(aLocation2[1]) - float(aLocation1[1])
    # bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    # if bearing < 0:
    #     bearing += 360.00

    #Define the ellipsoid
    geod = Geodesic.WGS84

    # Solve the Inverse problem
    inv = geod.Inverse(aLocation1[0],aLocation1[1],aLocation2[0],aLocation2[1])
    azi = inv['azi1']
    if azi < 0:
        azi += 360.00
    return azi

def getBearingGimbal(aLocation1, aLocation2):
    geod = Geodesic.WGS84

    # Solve the Inverse problem
    inv = geod.Inverse(aLocation1[0],aLocation1[1],aLocation2[0],aLocation2[1])
    azi = inv['azi1']
    if azi < 0:
        azi += 360.00
    return azi

def getTilt(aLocation1, aLocation2):
    # Returns the tilt angle between 2 altitudes.
    #print("atl1:",aLocation1[2])
    #print("atl2:",aLocation2[2])
    off_y = getDistanceMetres(aLocation1, aLocation2)  # distance from drone to the ROI
    off_x = aLocation2[2] - aLocation1[2]   # delta alt
    
    tilt = math.atan2(abs(off_x), abs(off_y))
    tilt = math.degrees(tilt)
    if off_x < 0: # lower pos
        tilt = -tilt 
    # if tilt < 0:
    #     tilt += 360.00
    
    return tilt

def sendNedVelocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
def sendGlobalVelocity(velocity_x, velocity_y, velocity_z, duration):

    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)   

def capturePhoto():
    """
    Send capture signal to camera
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
        0,
        1, 0 , 0 , 0 , 1 , 0,0) # param5 : trigger camera 
    # while True:
    #     # Wait for ACK command
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()

    #     # Print the ACK result !
    #     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    #     break

    print("Capture command sent")

def gimbalRotate(pitch = 0, roll=0, yaw = 0):
    gimbal_threshold = 2    
    master.mav.mount_configure_send(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,  #mount_mode
            1,  # stabilize roll
            1,  # stabilize pitch
            1,  # stabilize yaw
        )
    # while True: 
    #     # Wait for ACK command
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()
    #     break
    #print("******************************************")
    #print("Rotate gimbal to:",round(pitch,3),round(roll,3),round(yaw,3)) # ---------------------------------------------------- DEBUG
    #print("******************************************")
    master.mav.mount_control_send(
            0, 1,    # target system, target component
            int(pitch * 100),  # pitch is in centidegrees
            int(roll * 100),  # roll
            int(yaw * 100),  # yaw is in centidegrees
            0  # save position
        )
    # while True:
    #     # Wait for ACK command
    #     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    #     ack_msg = ack_msg.to_dict()

    #     # Print the ACK result !
    #     #print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    #     break

def waitUntilGuided():
    global current_mode 
    while turn_off == 0:
        msg = master.recv_match(type = 'HEARTBEAT', blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            #print(mode)
            if mode == 'GUIDED':
                current_mode = 'GUIDED'
                break
        
    print("GUIDED MODE ACTIVATED")

def checkCurrentMode():
    global current_mode 
    while turn_off == 0:
        try:
            msg = master.recv_match(type = 'HEARTBEAT', blocking = False)
            if msg and msg.type == 2:
                current_mode = mavutil.mode_string_v10(msg)
                if current_mode != 'GUIDED':
                    print("MODE: ",current_mode)
            time.sleep(1)
        except:
            pass

def getCurrentPos():
    global curr_loc,curr_loc_heading,curr_loc_goto,curr_loc_checkpos, lock,msg_reic_rate, msg_update_rate
    print("START GET CURRENT POS")
    temp_lat = 0
    temp_lng = 0
    temp_alt = 0
    temp_heading = 0
    MSL_alt = 0
    AGL_alt =  0
    terrain_height = 0
    GPS_rate = 0
    TERRAIN_rate = 0

    time_start = time.time()
    time_start_GPS = time.time()
    time_start_terrain = time.time()
    time_start_update = time.time()
    time.sleep(0.5)

    message = None
    print("START GETTING MSG")
    while turn_off == 0:

        while message is None: 
            try:
                message = master.recv_msg()
                msg_type =  message.get_type()
                break
            except:
                pass

        if msg_type == 'GLOBAL_POSITION_INT' or msg_type == 'TERRAIN_REPORT':
            if msg_type == 'GLOBAL_POSITION_INT':
                try:
                    GPS_rate = 1/(time.time() - time_start_GPS)
                except:
                    continue
                
                time_start_GPS = time.time()
                temp_lat = message.lat/10e6
                temp_lng = message.lon/10e6
                temp_alt =message.relative_alt/1000 # alt in mm
                temp_heading = message.hdg/100 # heading in cdeg
                MSL_alt = message.alt/1000 # alt in mm

            if msg_type == 'TERRAIN_REPORT':
                try:
                    TERRAIN_rate = 1/(time.time() - time_start_terrain)
                except:
                    continue
                
                time_start_terrain = time.time()
                AGL_alt = message.current_height
                terrain_height = message.terrain_height

        if time.time() - time_start_update > (1/(10*msg_update_rate)):
            while True: # try take latest info
                try:
                    curr_loc = (temp_lat,temp_lng,temp_alt,MSL_alt,AGL_alt,terrain_height,temp_heading)
                    time_start_update = time.time()
                    break
                except:
                    pass
            while True: # try update for navigation thread
                try:
                    curr_loc_heading = curr_loc
                    break
                except:
                    pass
            while True: # try update for navigation thread
                try:
                    curr_loc_goto = curr_loc 
                    break
                except:
                    pass
            while True: # try update for photo capture thread
                try:
                    curr_loc_checkpos = curr_loc 
                    break
                except:
                    pass
            while True: # try update for photo capture thread
                try:
                    curr_loc_gimbal = curr_loc 
                    break
                except:
                    pass

        if time.time() - time_start > 10:
            print(" ")
            print("GPS rate:",GPS_rate)
            print("TERRAIN rate:",TERRAIN_rate)
            print("POS:",curr_loc)
            print("POS_goto",curr_loc_goto)
            print("POS_heading",curr_loc_heading)
            print("POS_checkpos",curr_loc_checkpos)
            print("POS_gimbal",curr_loc_gimbal)
            print(" ")
            #print(message)
            time_start = time.time()

        message = None
# **************************************************************************
# *******************************************     POINT CAMERA TOWARD     **                                      
# **************************************************************************
# This function command the camera to point toward a position (lat,lon,alt)
def pointCameraToward(lat =0, lng =0, alt=0): # legacy - confict message lead to stuck

    GPS = master.location(relative_alt= True)
    #print ("Heading ", GPS.heading)
    pitch,roll,yaw = 0, 0, 0
    currentLocation = master.location(relative_alt= True)
    targetLocation = (float(lat),float(lng),float(alt))

    temp_lat = currentLocation.lat
    temp_lng = currentLocation.lng
    #temp_alt = currentLocation.alt
    AGL_alt,terrain_height = takeAGL()
    temp_alt = AGL_alt
    currentLocation = (temp_lat, temp_lng, temp_alt)

    yaw = getBearingGimbal(currentLocation, targetLocation)
    tilt = getTilt(currentLocation, targetLocation)
    roll = 0 

    yaw = yaw - GPS.heading
    if yaw < 0 :
        yaw += 360
    gimbalRotate(tilt,roll,yaw)
    #time.sleep(2)

def pointCameraToward_v2(currentPos,target):
    global curr_loc_gimbal,missed_waypoints
    next_ = False
    #print ("Heading ", GPS.heading)
    tilt,roll,yaw = 0, 0, 0

    targetLocation = (float(target[0]),float(target[1]),float(target[2]))

    temp_lat = currentPos[0]
    temp_lng = currentPos[1]
    #temp_alt = currentLocation.alt
    temp_alt = currentPos[2]
    currentLocation_temp = (temp_lat, temp_lng, temp_alt)

    yaw = getBearingGimbal(currentLocation_temp, targetLocation)
    tilt = getTilt(currentLocation_temp, targetLocation)
    roll = 0 

    yaw = yaw - currentPos[3]
    if yaw < 0 :
        yaw += 360
    if yaw > 170 and yaw < 190: # camera is missing 1 point, go to next waypoint
        next_ = True
        print(" ")
        print("!!!!!!!!!!!!!!--------!!!!!!!!!!!!!!")
        print("MISS A WAYPOINT, move to next")
        print("!!!!!!!!!!!!!!--------!!!!!!!!!!!!!!")
        print(" ")
        missed_waypoints +=1
    
    gimbalRotate(tilt,roll,yaw)
    time.sleep(1/(msg_update_rate))

    return next_

def rePos(pos=(0,0,0),heading =0):
    # this function will try to control the drone to move backward 2m then return to last position. 
    # this is to avoid being stuck at a position which cube's threshold is  higher than control signal.
    temp_azi = heading - 180 
    if temp_azi < 0:
        temp_azi += 360
    temp_pos_backward = calGPS1Point((pos[0],pos[1]), temp_azi, 2)
    print(temp_pos_backward)
    goToLowAcc(temp_pos_backward[0], temp_pos_backward[1], pos[2], heading)
    time.sleep(0.5)
    goTo(pos[0],pos[1],pos[2],heading)
    time.sleep(0.5)

# ***************************************************************************************************************************
# **************************************************************************     UTILS BLOCK    *****************************                                    
# *************************************************************************************************************************** 
#                                                                                                                          **
# This block content utilitity functions:                                                                                  **                                                                                         
#   Signal handler                                                                                                         **
#   Check Captured                                                                                                         **
#   Mission Scripts generate                                                                                               **
#   Read GPS file                                                                                                          **
#                                                                                                                          **
# ***************************************************************************************************************************
# ***************************************************************************************************************************

# **************************************************************************
# *********************************************     SIGNAL HANDLE     ******                                      
# ************************************************************************** 
from signal import signal, SIGINT

def handler(signal_received, frame):
    """
    This function handle ctrl+c command from user to stop the program.
    """
    global turn_off
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    print('Return to launch')
    turn_off = 1
    RTL()
    master.close()
    sys.exit(0)

# **************************************************************************
# *********************************************     CHECK CAPTURED     *****                                      
# ************************************************************************** 
# This function should be connect to storage to check captured photo
# exits and make sure the camera signal has been sent.

def checkCaptured():
    return 1

def checkPosReached(lat=0,lng =0,alt =0):
    # this function check if the drone reached a target
    global curr_loc_checkpos
    threshold = 1 # 1 meter radius accuracy
    targetLocation = (lat,lng,alt)
    while True:
        try:
            currentLocation = curr_loc_checkpos
            break
        except:
            pass
    temp_lat = currentLocation[0]
    temp_lng = currentLocation[1]
    temp_alt = currentLocation[4] # AGL alt
    temp_heading = currentLocation[6]

    loc1 = (temp_lat,temp_lng,temp_alt)
    loc2 = (lat,lng,alt)
    currPos = (temp_lat,temp_lng,temp_alt,temp_heading)
    targetDistance = getDistanceMetres(loc1, loc2)
    remainingDistance = round(getDistanceMetres(loc1, loc2),3)
    remainingAltitude = round((temp_alt - alt),3)
    #print( "Thread camera, Distance to target: ", remainingDistance," |-| Altitude to target: ", remainingAltitude) # ---------------------------------------------------- DEBUG
    if (remainingDistance<= threshold ) and abs(temp_alt - targetLocation[2])< 10*threshold  : # If the drone reachs target, then disband the check
        print("Thread camera: Target at ",round(targetDistance,3),"m away"," altitude ",round(alt,3),"m reached")
        #print(" ")
        return True, currPos
    else:
        return False, currPos

# **************************************************************************
# *****************************************     MISSION SCRIPTS PYLON     **                                      
# **************************************************************************
# This function generate action script at pylons

def missionScriptPylon(inspection_pos = (0,0,0), heading = 180, gimbal_pos = (0,0,0)): #xxx_pos : lat,lon,alt
    """
    This function is activated when a pylon waypoint is reached.
    It will capture the pylon photos in 5 different altitude.
    """
    global total_capture
    time_start = time.time()
    print("Set speed to: ",mission_speed)
    setSpeed(mission_speed)
    lat = inspection_pos[0]
    lng = inspection_pos[1]
    alt = inspection_pos[2]

    gimbal_lat = gimbal_pos[0]
    gimbal_lng = gimbal_pos[1]
    gimbal_alt = gimbal_pos[2]

    i=0
    while i<1:
        goTo(lat,lng,(alt-alt*i*0.1),heading)
        #print("Go to: ",lat, lng, alt, "heading :",heading)
        pointCameraToward( gimbal_lat, gimbal_lng,(gimbal_alt- gimbal_alt*i*0.1)) # point gimbal to pylon
        print("Point gimbal to:",gimbal_pos)
        print("Capture photos ",i+1," of pylon")
        time.sleep(1)
        capturePhoto()
        total_capture +=1
        check_count = 0
        while True:
            # Wait for ACK command
            check_photo = checkCaptured()
            if check_photo ==1:
                break
            else:
                if check_count == 5:
                    check_count = 0
                    print("Re-capture photo")
                    # re capture the photo
                    pointCameraToward( gimbal_lat, gimbal_lng,(gimbal_alt- gimbal_alt*i*0.1)) # point gimbal to pylon
                    print("GIMBAL POSITION:",gimbal_pos)
                    goTo(lat,lng,(alt-alt*i*0.1),heading)
                    capturePhoto()
                else:    
                    check_count +=1   
            time.sleep(0.5)
        i+=1

    #gimbalRotate(0,0,0)  # return gimbal to forward
    goTo(lat,lng,alt,heading) # return to inspection altitude
    print(" ")
    print("Inspection pylon time:",round((time.time()-time_start),3))
    print(" ")

# **************************************************************************
# ******************************************     MISSION SCRIPTS LINE     **                                      
# **************************************************************************
# This function generate action script at powerline points

def missionScriptLine(inspection_pos = (0,0,0), heading = 180, gimbal_pos = (0,0,0)): #xxx_pos : lat,lon,alt
    """
    This function is activated when a powerline waypoint is reached.
    It will capture 1 photo only.
    """
    global total_capture,mision_speed

    lat = inspection_pos[0]
    lng = inspection_pos[1]
    alt = inspection_pos[2]

    gimbal_lat = gimbal_pos[0]
    gimbal_lng = gimbal_pos[1]
    gimbal_alt = gimbal_pos[2]

    time_start = time.time()
    print("Set speed to: ",mission_speed)
    setSpeed(mission_speed)
    goTo(lat,lng,alt,heading)
    pointCameraToward(gimbal_lat, gimbal_lng,gimbal_alt) # # point gimbal to the line
    print("Point gimbal to:",gimbal_pos)
    time.sleep(1)

    capturePhoto()
    total_capture +=1
    check_count = 0
    while True:
        # Wait for ACK command
        check_photo = checkCaptured()
        if check_photo ==1:
            break
        else:
            if check_count == 5:
                check_count = 0
                # re capture the photo
                goTo(lat,lng,alt,heading)
                pointCameraToward(gimbal_lat, gimbal_lng,gimbal_alt) # # point gimbal to the line
                capturePhoto()
            else:    
                check_count +=1   
        time.sleep(0.5)

    #gimbalRotate(0,0,0)  # return gimbal to forward
    print("Inspection powerline time:",round((time.time()-time_start),3))

# **************************************************************************
# ******************************************************     READ GPS     **                                 
# ************************************************************************** 
# This function read GPS file contains pylons locations

def readGPS(file_path = 'Pylons_GPS.txt'):
    file = open(file_path, 'r')
    lines = file.readlines()
    pylons = []
    # Spit lines into lists
    for line in lines:
        #print(line)
        #print(line.split())
        pylons.append(line.split())
    print("-------------------------------")
    print("PYLONS:",len(pylons))
    print("-------------------------------")
    return pylons

# **************************************************************************
# *********************************************     WAYPOINT FILE CONVERT **                                     
# ************************************************************************** 
# This function convert *.waypoints generated by MP to *.txt

def waypointFileConvert(file_path = 'Pylons_GPS_cnc.waypoint' ):
    "This function convert *.waypoint files to .txt"
    file = open(file_path, 'r')
    lines = file.readlines()
    pylons = []
    line_count = 1
    # Spit lines into lists
    while line_count < len(lines):
        #print(lines[line_count].split())
        pylons.append(lines[line_count].split())
        line_count += 1
    i = 0
    with open(file_path+'.txt', 'w') as f:
        while i< len(pylons):
            f.write(pylons[i][8]+" "+pylons[i][9]+" "+pylons[i][10]+"\n")
            i+=1
        i = -1
        # add rally points to turn back
        f.write(pylons[i][8]+" "+pylons[i][9]+" "+pylons[i][10]+"\n")
        f.write(pylons[i][8]+" "+pylons[i][9]+" "+pylons[i][10]+"\n")

        # add backward points
        while i > (-len(pylons))-1:
            f.write(pylons[i][8]+" "+pylons[i][9]+" "+pylons[i][10]+"\n")
            i-=1
        f.close()
    return (file_path+'.txt')

# **************************************************************************
# ********************************     GENERATE PYLON INSPECTION WAYPOINT **                                     
# ************************************************************************** 
# This function generate Inspection position for drone based on pylon's GPS

def generatePylonInspectionWaypoints(pylons =[],inspection_distance = 10,yaw_thresold = 0):
    i = 0
    inspection_waypoint = []
    safety_altitude = 50
    # Create inspection positions:
    while i < len(pylons):
        if i == 0: # auto mision home rally point
            A = (float(pylons[i][0]),float(pylons[i][1]))
            B = (float(pylons[i-1][0]),float(pylons[i+1][1]))
            azi = getBearing(A,B)
            alt = float(pylons[i][2])
            if azi < 0:
                azi += 360
            azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to take previous pylon as reference
            inspection_waypoint.append((lat_temp,lng_temp, alt,azi_temp)) # lat,long,alt,azimuth

        elif i == 1: # first rally point
            A = (float(pylons[i][0]),float(pylons[i][1]))
            B = (float(pylons[i+1][0]),float(pylons[i+1][1]))
            azi = getBearing(A,B)
            alt = float(pylons[i][2])
            if azi < 0:
                azi += 360
            azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to take previous pylon as reference
            inspection_waypoint.append((lat_temp,lng_temp, alt,azi_temp)) # lat,long,alt,azimuth

        elif i == len(pylons)/2 - 2: # rallypoint back low, using previous pylon's GPS for reference
            A = (float(pylons[i][0]),float(pylons[i][1]))
            B = (float(pylons[i-1][0]),float(pylons[i-1][1]))
            azi = getBearing(B,A)
            alt = float(pylons[i][2])
            if azi < 0:
                azi += 360
            azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to take previous pylon as reference
            inspection_waypoint.append((lat_temp,lng_temp, alt,azi_temp)) # lat,long,alt,azimuth
            print(i,inspection_waypoint[i])

        elif i == len(pylons)/2 -1: # turn back points 1 (high)
            # A = (float(pylons[i-1][0]),float(pylons[i-1][1]))
            # B = (float(pylons[i-2][0]),float(pylons[i-2][1]))
            # azi = getBearing(B,A)
            alt = float(pylons[i-1][2]) + safety_altitude # higher 100 meter for sure
            # if azi < 0:
            #     azi += 360
            # azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to take previous pylon as reference
            inspection_waypoint.append((lat_temp,lng_temp, alt,azi_temp)) # lat,long,alt,azimuth
            print(i,inspection_waypoint[i])

        elif i == len(pylons)/2: # turn back points 2 (high)
            A = (float(pylons[i-2][0]),float(pylons[i-2][1]))
            B = (float(pylons[i-3][0]),float(pylons[i-3][1]))
            azi = getBearing(A,B) # turn around
            alt = float(pylons[i-2][2]) + safety_altitude # higher 100 meter for sure
            if azi < 0:
                azi += 360
            azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to turned around
            inspection_waypoint.append((lat_temp,lng_temp, alt,azi_temp)) # lat,long,alt,azimuth
            print(i,inspection_waypoint[i])

        elif i == len(pylons)/2 +1: # turn back points 3 (low)
            # A = (float(pylons[i-2][0]),float(pylons[i-2][1]))
            # B = (float(pylons[i-3][0]),float(pylons[i-3][1]))
            # azi = getBearing(A,B) # turn around
            alt = float(pylons[i-3][2]) # down 100 meter to opposite line
            # if azi < 0:
            #     azi += 360
            # azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to turned around
            inspection_waypoint.append((lat_temp,lng_temp, alt,azi_temp)) # lat,long,alt,azimuth
            print(i,inspection_waypoint[i])

        elif i == len(pylons)-2: # get home rally point
            A = (float(pylons[i][0]),float(pylons[i][1]))
            B = (float(pylons[i-1][0]),float(pylons[i-1][1]))
            azi = getBearing(B,A)
            alt = float(pylons[i][2]) + safety_altitude # higher 100 meter for sure
            if azi < 0:
                azi += 360
            azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to take previous pylon as reference
            inspection_waypoint.append((lat_temp,lng_temp,alt,azi_temp)) # lat,long,alt,azimuth

        elif i == len(pylons)-1: #  last rally point
            A = (float(pylons[i][0]),float(pylons[i][1]))
            B = (float(pylons[i-1][0]),float(pylons[i-1][1]))
            azi = getBearing(B,A)
            alt = float(pylons[i][2]) + safety_altitude # higher 100 meter for sure
            if azi < 0:
                azi += 360
            azi_temp = azi
            lat_temp,lng_temp = calGPS1Point90Degree(A,azi,inspection_distance)  # reverse -90 due to take previous pylon as reference
            inspection_waypoint.append((lat_temp,lng_temp,alt,azi_temp)) # lat,long,alt,azimuth


        else: # other pylons
            A = (float(pylons[i-1][0]),float(pylons[i-1][1])) #rally point will take last pylon azimuth to rotate
            B = (float(pylons[i][0]),float(pylons[i][1]))
            C=  (float(pylons[i+1][0]),float(pylons[i+1][1]))
            pylons[i][2]
            alt = float(pylons[i][2])
            lat_temp,lng_temp,azi_temp = calGPS3Points90Degree(A,B,C,inspection_distance,yaw_thresold) #azimuth use to rotate the drone
            inspection_waypoint.append((lat_temp,lng_temp,alt,azi_temp))
        i+=1

        # go back position:
    return inspection_waypoint

# **************************************************************************
# **************************     GENERATE PYLON INSPECTION WAYPOINT DEBUG **                                     
# ************************************************************************** 
# This function used for debug of generate 
# Inspection position for drone based on pylon's GPS

def generatePylonInspectionWaypointsDebug(pylons=[]):
    waypoints = []
    lowest_point = 5
    count = 0
    i = 0
    while i< len(pylons):
        print("Waypoint ",count," generated")

        if i == 0: # rally point
            pylon_lat = float(pylons[i][0])
            pylon_lng = float(pylons[i][1])
            pylon_alt = float(pylons[i][2])

            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt))

            count+=1
    # first pylon:
        elif i == 1:
            pylon_lat = float(pylons[i][0])
            pylon_lng = float(pylons[i][1])
            pylon_alt = float(pylons[i][2])

            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt))

            count +=1

    # last pylon:
        elif i == len(pylons) -1:
            pylon_lat = float(pylons[i][0])
            pylon_lng = float(pylons[i][1])
            pylon_alt = float(pylons[i][2])

            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt))

            count +=1

    # other pylons:
        else:
            pylon_lat = float(pylons[i][0])
            pylon_lng = float(pylons[i][1])
            pylon_alt = float(pylons[i][2])

            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt))

            count +=1  
        i+=1

    for i in waypoints:
        print("WAYPOINTS:", i)
    return waypoints

# **************************************************************************
# ****************************     GENERATE POWER LINE INSPECTION MISSION **                                     
# ************************************************************************** 
# This is the main function to generate the mission's waypoints.
# This function take in Inspection Waypoints of drone generated
# by the GENERATE PYLON INSPECTION WAYPOINT function and generate
# additionals waypoint to capture the pylon and line photos.

def generatePowerlineInspectionMission(pylons=[],inspection_points=[],line_point = 3,inspection_distance = 10 ):
    global safety_distance
    violation = 0
    waypoints = []
    gimbal_waypoints = []
    lowest_point = 5
    count = 0
    distance = inspection_distance
    #distance = math.tan(math.pi/4)*inspection_distance
    print("---------------------------------------------------------------")
    print("-------------------  GENERATING MISSION  ----------------------")
    print("---------------------------------------------------------------")
    # math.tan(45) * distance
    i = 0
    while i< len(pylons):
    # rally point    
        if i == 0 or i == 1: 
            print("rally")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])
            
            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            gimbal_waypoints.append(("rally",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
            count+=1

    # first pylon:
        elif i == 2:
            print("first pylon")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position
            gimbal_lng = float(pylons[i][1])
            gimbal_alt = float(pylons[i][2])

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1])) # previous point
            B = (float(inspection_points[i][0]),float(inspection_points[i][1])) # current point
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1])) # next point
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1])
            
            azi_temp_45 = round(getBearing(A,B),7)
            pylon_45_lat,pylon_45_lng = calGPS1Point(B,azi_temp_45,-distance)
            new_distance = -distance

            azi_temp_135 = round(getBearing(B,C),7)
            pylon_135_lat,pylon_135_lng = calGPS1Point(B,azi_temp_135,distance) 
            new_distance = distance

            waypoints.append(("pylon_45",pylon_45_lat,pylon_45_lng,pylon_alt,azi_temp_45)) # add inspection waypoints to the list
            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            waypoints.append(("pylon_135",pylon_135_lat,pylon_135_lng,pylon_alt,azi_temp_135))
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add gimbal pointing waypoints to the list
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add

            count +=1

    # end point:
        elif i == len(pylons) - 1 or i == len(pylons) - 2 :
            print("rally end")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])
            
            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            gimbal_waypoints.append(("rally",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
            count +=1

    # backward course rally points:
        elif i == len(pylons)/2 -2 or i == len(pylons)/2 -1 or i == len(pylons)/2 or i == len(pylons)/2 +1 :
            print("rally backward")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])
            
            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            gimbal_waypoints.append(("rally",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
            count +=1

    # First turn around pylon
        elif i == len(pylons)/2 + 2:
            print("first pylon backward")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1]))
            B = (float(inspection_points[i][0]),float(inspection_points[i][1]))
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1]))
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1]) 

            # Distance beetwen 2 line point
            line_distance = calDistance2points(A,B)/(line_point+1) 

            azi_temp_45 = round(getBearing(A,B),7)
            pylon_45_lat,pylon_45_lng = calGPS1Point(B,azi_temp_45,-distance)
            new_distance = -distance

            azi_temp_135 = round(getBearing(B,C),7)
            pylon_135_lat,pylon_135_lng = calGPS1Point(B,azi_temp_135,distance) 
            new_distance = distance

            waypoints.append(("pylon_45",pylon_45_lat,pylon_45_lng,pylon_alt,azi_temp_45))
            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            waypoints.append(("pylon_135",pylon_135_lat,pylon_135_lng,pylon_alt,azi_temp_135))
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add gimbal pointing waypoints to the list
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add
            count +=1

    # other pylons:
        else:
            print("pylon ",i)
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position
            gimbal_lng = float(pylons[i][1])
            gimbal_alt = float(pylons[i][2])

            # ---------------------------------------------------------    Init pylon GPS

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1]))
            B = (float(inspection_points[i][0]),float(inspection_points[i][1]))
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1])) # next point
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1]) 

            # Distance beetwen 2 line point
            line_distance = calDistance2points(A,B)/(line_point+1) 
            
            j = 1
            # Create point between 2 pylons
            while j <= line_point:
                line_lat,line_lng = calGPS2Points(A,B,line_distance*j)
                if j == int((line_point//2 +1)): # mid line point
                    line_alt = float(pylon_alt) - lowest_point
                else:
                    line_alt = float(pylon_alt) - lowest_point/2
                j+=1
                line_azi = getBearing(A,B)
                line_azi = round(line_azi,7)
                waypoints.append(("line",line_lat,line_lng,line_alt,line_azi))

                count +=1

            azi_temp_45 = round(getBearing(A,B),7)
            pylon_45_lat,pylon_45_lng = calGPS1Point(B,azi_temp_45,-distance)
            new_distance = -distance
            
            azi_temp_135 = round(getBearing(B,C),7)
            pylon_135_lat,pylon_135_lng = calGPS1Point(B,azi_temp_135,distance) 
            new_distance = distance

            #----------------------------------------------------------    Init gimbal pointing GPS

            A = (float(pylons[i-1][0]),float(pylons[i-1][1]))
            B = (float(pylons[i][0]),float(pylons[i][1]))
            # Distance beetwen 2 line point
            line_distance = calDistance2points(A,B)/(line_point+1) 

            j = 1
            # Create gimbal pointing point between 2 pylons
            while j <= line_point:
                line_lat,line_lng = calGPS2Points(A,B,line_distance*j)
                if j == int((line_point//2 +1)): # mid line point
                    line_alt = float(gimbal_alt) - lowest_point
                else:
                    line_alt = float(gimbal_alt) - lowest_point/2
                j+=1
                line_azi = pylon_azi + 90 # inverse with azi4 in the calculation
                gimbal_waypoints.append(("line",line_lat,line_lng,line_alt))

            # ----------------------------------------------------------    Add waypoints

            waypoints.append(("pylon_45",pylon_45_lat,pylon_45_lng,pylon_alt,azi_temp_45))
            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            waypoints.append(("pylon_135",pylon_135_lat,pylon_135_lng,pylon_alt,azi_temp_135))
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add gimbal pointing waypoints to the list
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add
            gimbal_waypoints.append(("pylon",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add

            count +=1
        i+=1

    print("                ",count," Waypoint generated")
    print("---------------------------------------------------------------")
    for i in waypoints:
        print("WAYPOINTS:", i)
    print("---------------------------------------------------------------")
    print("WAYPOINTS:",len(waypoints))
    print("GIMBAL WAYPOINTS:",len(gimbal_waypoints))
    # for waypoint in waypoints:
    #     print(waypoint)
    # for gimbal_waypoint in gimbal_waypoints:
    #     print(gimbal_waypoint)
    return waypoints,gimbal_waypoints

def generatePowerlineInspectionMission_v2(pylons=[],inspection_points=[],line_point = 3,inspection_distance = 10 ):
    global safety_distance, pylon_point
    waypoints = []
    gimbal_waypoints = []
    lowest_point = 5
    count = 0
    distance = inspection_distance
    #distance = math.tan(math.pi/4)*inspection_distance
    print("---------------------------------------------------------------")
    print("-------------------  GENERATING MISSION  ----------------------")
    print("---------------------------------------------------------------")
    # math.tan(45) * distance
    i = 0
    while i< len(pylons):
    # rally point    
        if i == 0 or i == 1 or i == -1: 
            print("rally")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[2][3]) # take the first pylon azi instead

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])
            
            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            gimbal_waypoints.append(("rally",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
            count+=1

    # first pylon:
        elif i == 2:
            print("first pylon")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position
            gimbal_lng = float(pylons[i][1])
            gimbal_alt = float(pylons[i][2])

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1])) # previous point
            B = (float(inspection_points[i][0]),float(inspection_points[i][1])) # current point
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1])) # next point
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1])
            
            j = 0 
            azi_temp = 0
            pylon_lat_temp = 0
            pylon_lng_temp = 0
            pylon_inspection_points_distance = distance/(pylon_point//2)
            while j < pylon_point//2:
                azi_temp = round(getBearing(A,B),7)
                pylon_lat_temp,pylon_lng_temp = calGPS1Point(B,azi_temp, -(distance - j*pylon_inspection_points_distance))
                waypoints.append(("pylon_45",pylon_lat_temp,pylon_lng_temp,pylon_alt,azi_temp)) # add inspection waypoints to the list
                gimbal_waypoints.append(("pylon_45",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
                j+=1

            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt,pylon_azi)) # mid pylon inspection point
            gimbal_waypoints.append(("pylon_90",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add gimbal pointing waypoints to the list

            j = 1 
            azi_temp = 0
            pylon_lat_temp = 0
            pylon_lng_temp = 0
            pylon_inspection_points_distance = distance/(pylon_point//2)
            while j <= pylon_point//2:
                azi_temp = round(getBearing(B,C),7)
                pylon_lat_temp,pylon_lng_temp = calGPS1Point(B,azi_temp,j*pylon_inspection_points_distance)
                waypoints.append(("pylon_135",pylon_lat_temp,pylon_lng_temp,pylon_alt,azi_temp)) # add inspection waypoints to the list
                gimbal_waypoints.append(("pylon_135",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
                j+=1

            count +=1

    # end point:
        elif i == len(pylons) - 1 or i == len(pylons) - 2 :
            print("rally end")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])
            
            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            gimbal_waypoints.append(("rally",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
            count +=1

    # backward course rally points:
        elif i == len(pylons)/2 -2 or i == len(pylons)/2 -1 or i == len(pylons)/2 or i == len(pylons)/2 +1 :
            print("rally backward")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3]) # take the first turn around pylon azimuth

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])
            
            waypoints.append(("rally",pylon_lat,pylon_lng,pylon_alt,pylon_azi))
            gimbal_waypoints.append(("rally",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
            count +=1

    # First turn around pylon
        elif i == len(pylons)/2 + 2:
            print("first pylon backward")
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position at rally point isn't used in fact, it's generated 
            gimbal_lng = float(pylons[i][1]) # for easier counting.
            gimbal_alt = float(pylons[i][2])

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1]))
            B = (float(inspection_points[i][0]),float(inspection_points[i][1]))
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1]))
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1]) 

            j = 0 
            azi_temp = 0
            pylon_lat_temp = 0
            pylon_lng_temp = 0
            pylon_inspection_points_distance = distance/(pylon_point//2)
            while j < pylon_point//2:
                azi_temp = round(getBearing(A,B),7)
                pylon_lat_temp,pylon_lng_temp = calGPS1Point(B,azi_temp, -(distance - j*pylon_inspection_points_distance))
                waypoints.append(("pylon_45",pylon_lat_temp,pylon_lng_temp,pylon_alt,azi_temp)) # add inspection waypoints to the list
                gimbal_waypoints.append(("pylon_45",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
                j+=1

            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt,pylon_azi)) # mid pylon inspection point
            gimbal_waypoints.append(("pylon_90",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add gimbal pointing waypoints to the list

            j = 1 
            azi_temp = 0
            pylon_lat_temp = 0
            pylon_lng_temp = 0
            pylon_inspection_points_distance = distance/(pylon_point//2)
            while j <= pylon_point//2:
                azi_temp = round(getBearing(B,C),7)
                pylon_lat_temp,pylon_lng_temp = calGPS1Point(B,azi_temp,j*pylon_inspection_points_distance)
                waypoints.append(("pylon_135",pylon_lat_temp,pylon_lng_temp,pylon_alt,azi_temp)) # add inspection waypoints to the list
                gimbal_waypoints.append(("pylon_135",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
                j+=1

            count +=1

    # other pylons:
        else:
            print("pylon ",i)
            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position
            gimbal_lng = float(pylons[i][1])
            gimbal_alt = float(pylons[i][2])

            # ---------------------------------------------------------    Init pylon GPS

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1]))
            B = (float(inspection_points[i][0]),float(inspection_points[i][1]))
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1])) # next point
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1]) 

            # Distance beetwen 2 line point
            line_distance = calDistance2points(A,B)/(line_point+1) 
            
            j = 1
            # Create point between 2 pylons
            while j <= line_point:
                line_lat,line_lng = calGPS2Points(A,B,line_distance*j)
                if j == int((line_point//2 +1)): # mid line point
                    line_alt = float(pylon_alt) - lowest_point
                else:
                    line_alt = float(pylon_alt) - lowest_point/2
                j+=1
                line_azi = getBearing(A,B)
                line_azi = round(line_azi,7)
                waypoints.append(("line",line_lat,line_lng,line_alt,line_azi))

                count +=1
            #----------------------------------------------------------    Init gimbal pointing GPS

            A = (float(pylons[i-1][0]),float(pylons[i-1][1]))
            B = (float(pylons[i][0]),float(pylons[i][1]))
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1])) # next point
            # Distance beetwen 2 line point
            line_distance = calDistance2points(A,B)/(line_point+1) 

            j = 1
            # Create gimbal pointing point between 2 pylons
            while j <= line_point:
                line_lat,line_lng = calGPS2Points(A,B,line_distance*j)
                if j == int((line_point//2 +1)): # mid line point
                    line_alt = float(gimbal_alt) - lowest_point
                else:
                    line_alt = float(gimbal_alt) - lowest_point/2
                j+=1
                line_azi = pylon_azi + 90 # inverse with azi4 in the calculation
                gimbal_waypoints.append(("line",round(line_lat,7), round(line_lng,7), round(line_alt,7)))
            # ----------------------------------------------------------------------------------------

            pylon_lat = float(inspection_points[i][0]) # inspection position
            pylon_lng = float(inspection_points[i][1])
            pylon_alt = float(inspection_points[i][2])
            pylon_azi = float(inspection_points[i][3])

            gimbal_lat = float(pylons[i][0]) # gimbal pointing position
            gimbal_lng = float(pylons[i][1])
            gimbal_alt = float(pylons[i][2])

            A = (float(inspection_points[i-1][0]),float(inspection_points[i-1][1])) # previous point
            B = (float(inspection_points[i][0]),float(inspection_points[i][1])) # current point
            C = (float(inspection_points[i+1][0]),float(inspection_points[i+1][1])) # next point
            A_pylon = float(pylons[i-1][0]),float(pylons[i-1][1])
            B_pylon = float(pylons[i][0]),float(pylons[i][1])
            C_pylon = float(pylons[i+1][0]),float(pylons[i+1][1])
            
            j = 0 
            azi_temp = 0
            pylon_lat_temp = 0
            pylon_lng_temp = 0
            pylon_inspection_points_distance = distance/(pylon_point//2)
            while j < pylon_point//2:
                azi_temp = round(getBearing(A,B),7)
                pylon_lat_temp,pylon_lng_temp = calGPS1Point(B,azi_temp, -(distance - j*pylon_inspection_points_distance))
                waypoints.append(("pylon_45",pylon_lat_temp,pylon_lng_temp,pylon_alt,azi_temp)) # add inspection waypoints to the list
                gimbal_waypoints.append(("pylon_45",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
                j+=1

            waypoints.append(("pylon_90",pylon_lat,pylon_lng,pylon_alt,pylon_azi)) # mid pylon inspection point
            gimbal_waypoints.append(("pylon_90",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7))) # add gimbal pointing waypoints to the list

            j = 1 
            azi_temp = 0
            pylon_lat_temp = 0
            pylon_lng_temp = 0
            pylon_inspection_points_distance = distance/(pylon_point//2)
            while j <= pylon_point//2:
                azi_temp = round(getBearing(B,C),7)
                pylon_lat_temp,pylon_lng_temp = calGPS1Point(B,azi_temp,j*pylon_inspection_points_distance)
                waypoints.append(("pylon_135",pylon_lat_temp,pylon_lng_temp,pylon_alt,azi_temp)) # add inspection waypoints to the list
                gimbal_waypoints.append(("pylon_135",round(gimbal_lat,7), round(gimbal_lng,7), round(gimbal_alt,7)))
                j+=1

            count +=1
        i+=1

    print("                ",count," Waypoint generated")
    print("---------------------------------------------------------------")
    for i in waypoints:
        print("WAYPOINTS:", i)
    print("---------------------------------------------------------------")
    print("WAYPOINTS:",len(waypoints))
    print("GIMBAL WAYPOINTS:",len(gimbal_waypoints))
    return waypoints,gimbal_waypoints

def sortWaypoints(waypoints = []):
    global turn_off
    i = 0
    j = 0
    pylon_90 = []
    temp_list = []
    print("------------------------------------------------------------------")
    print("SORTED WAYPOINTS")
    print("")

    while i< len(waypoints) and turn_off == 0:
        if waypoints[i][0] == 'pylon_90' : # mark all pylon_90
            pylon_90.append(i)
        i+=1
    i = 0

    while i<int(len(pylon_90)/2):
        j = pylon_90[i] # index of pylon90s in waypoints[]
        loc_0 = (waypoints[j][1],waypoints[j][2])
        if i == len(pylon_90)/2 -1: 
            i+=1
            break # last pylon doesnt need to sort
        else:
            j+=1 # start sorting from next point to pylon_90
            while j<pylon_90[i+1]: # take index from a pylon_90 to the next pylon_90
                loc_1 = (waypoints[j][1],waypoints[j][2])
                temp_element = {"id":j,                                      \
                                "data":waypoints[j],                         \
                                "dist":getDistanceMetres(loc_0,loc_1)} # calculate distance between waypoints and add to list
                temp_list.append(temp_element) 
                j+=1       
        # sort the list

        temp_list.sort(key=lambda x: x.get('dist'))
        print("")
        for temp_print in temp_list:
            print(temp_print)
        # return the sorted list to waypoints
        j = pylon_90[i]
        j+=1
        k = 0
        while j<pylon_90[i+1]:
            waypoints[j] = temp_list[k]['data']
            j+=1
            k+=1
        temp_list = []
        i+=1

    while i<int(len(pylon_90)):# backward pylon
        j = pylon_90[i] # index of pylon90s in waypoints[]
        loc_0 = (waypoints[j][1],waypoints[j][2])
        if i == len(pylon_90) -1: 
            break # last pylon doesnt need to sort
        else:
            j+=1 # start sorting from next point to pylon_90
            while j<pylon_90[i+1]: # take index from a pylon_90 to the next pylon_90
                loc_1 = (waypoints[j][1],waypoints[j][2])
                temp_element = {"id":j,                                      \
                                "data":waypoints[j],                         \
                                "dist":getDistanceMetres(loc_0,loc_1)} # calculate distance between waypoints and add to list
                temp_list.append(temp_element) 
                j+=1       
        # sort the list

        temp_list.sort(key=lambda x: x.get('dist'))
        print("")
        for temp_print in temp_list:
            print(temp_print)
        # return the sorted list to waypoints
        j = pylon_90[i]
        j+=1
        k = 0
        while j<pylon_90[i+1]:
            waypoints[j] = temp_list[k]['data']
            j+=1
            k+=1
        temp_list = []
        i+=1
    
    return waypoints

# **************************************************************************
# *********************************************     GPS CALCULATION ********                                      
# ************************************************************************** 
from geographiclib.geodesic import Geodesic

def calGPS3Points90Degree(A=(0,0),B=(0,0),C=(0,0),distance = 10,threshold = 0):
    # this function calculate the GPS position which is perpendicuar with tangent line at mid point of a triplet at a distance.
    global reverse
    # azi1 = A to B
    # azi2 = B to C
    # azi3 = A to C
    # azi4 = B to inspection Position
     #Define the ellipsoid
    geod = Geodesic.WGS84

    # Solve the Inverse problem
    inv = geod.Inverse(A[0],A[1],C[0],C[1])
    azi3 = inv['azi1']
    azi_return = azi3
    if azi_return < 0:
        azi_return += 360
    # Calculate azi4
    azi4 = azi3 + 270 + threshold  # for fine adjiustment
    if reverse == 1:
        azi4 = azi4 - 180
    if azi4 < 0 :
        azi4 +=360

    # Solve the Direct problem
    dir = geod.Direct(B[0], B[1], azi4, inspection_distance)
    D = (dir['lat2'],dir['lon2'])
     
    return round(D[0],7),round(D[1],7),round(azi_return,7)

def calGPS2Points90Degree(A=(10.8266951,106.8252427),B=(10.8266951,106.8252427),distance = 10):
    # this function calculate the GPS position which is perpendicular with the direction of 2 inspected pylon at a distance.
    global reverse
    s = distance 

    #Define the ellipsoid
    geod = Geodesic.WGS84

    #Solve the Inverse problem
    inv = geod.Inverse(A[0],A[1],B[0],B[1])
    azi1 = inv['azi1'] + 90
    if reverse == 1:
        azi1 = azi1 - 180
    if azi1 < 0 :
        azi1 +=360

    azi_return = inv['azi1']
    if azi_return < 0:
        azi_return += 360
    print('Initial Azimuth from A to B = ' + str(azi_return))

    #Solve the Direct problem
    dir = geod.Direct(A[0],A[1],azi1,s)
    C = (dir['lat2'],dir['lon2'])
    #print("Waypoint generated")

    return round(C[0],7),round(C[1],7),round(azi_return,7)

def calGPS2Points(A=(10.8266951,106.8252427),B=(10.8266951,106.8252427),distance = 10):
    s = distance #Distance (m)

    #Define the ellipsoid
    geod = Geodesic.WGS84

    #Solve the Inverse problem
    inv = geod.Inverse(A[0],A[1],B[0],B[1])
    azi1 = inv['azi1']
    #print('Initial Azimuth from A to B = ' + str(azi1))

    #Solve the Direct problem
    dir = geod.Direct(A[0],A[1],azi1,s)
    C = (dir['lat2'],dir['lon2'])
    #print("Waypoint generated")
    return round(C[0],7),round(C[1],7)

def calGPS1Point(A=(10.8266951,106.8252427),azi = 0,distance = 10):
    s = distance #Distance (m)

    #Define the ellipsoid
    geod = Geodesic.WGS84

    #Solve the Direct problem
    dir = geod.Direct(A[0],A[1],azi,s)
    C = (dir['lat2'],dir['lon2'])
    #print("Waypoint generated")
    return round(C[0],7),round(C[1],7)

def calGPS1Point90Degree(A=(10.8266951,106.8252427),azi = 0,distance = 10):
    global reverse
    s = distance #Distance (m)

    #Define the ellipsoid
    geod = Geodesic.WGS84

    azi = azi + 270
    if reverse == 1:
        azi = azi - 180

    #Solve the Direct problem
    dir = geod.Direct(A[0],A[1],azi,s)
    C = (dir['lat2'],dir['lon2'])
    #print("Waypoint generated")
    return round(C[0],7),round(C[1],7)

def calDistance2points(A=(10.8266951,106.8252427),B=(10.8266951,106.8252427)):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    deltaLat = B[0] - A[0]
    deltaLong = B[1] - A[1]
    #print(deltaLat,deltaLong)
    return math.sqrt((deltaLat*deltaLat) + (deltaLong*deltaLong)) * 1.113195e5

def checkCornerViolation(A=(0,0),B=(0,0),C=(0,0),inspection_pos=(), safe_distance = 7.5):
    # this function will check if the corner of a pylon is large enough for a safe inspection or not.
    disAB = abs(getDistanceMetres(A,B)) # check triangle AI1B
    disBI1 = abs(getDistanceMetres(B,inspection_pos))
    disAI1 = abs(getDistanceMetres(A,inspection_pos))
    p = (disAB + disBI1 + disAI1)/2
    disCheck = abs((2/disAB)* math.sqrt(abs(p*(p-disAB)*(p-disBI1)*(p-disAI1))))
    #print("DISCHECK 1 : ",disCheck)
    if disCheck > safe_distance: # check triangle BI1C if AI1B is fine
        disBC = abs(getDistanceMetres(B,C))
        disCI1 = abs(getDistanceMetres(C,inspection_pos))
        disBI1 = abs(getDistanceMetres(B,inspection_pos))
        p = (disBC + disBI1 + disCI1)/2
        disCheck = abs((2/disBC)* math.sqrt(abs(p*(p-disBC)*(p-disBI1)*(p-disCI1))))
        #print("DISCHECK2 : ",disCheck)
        if disCheck > safe_distance:
            return True
        else: 
            return False
    else: 
        return False

def cornerViolationEvade(A=(0,0),azi = 0,distance=0):
    # This function generate alternative for violated inspection positions
    alt_lat = 0
    alt_lon = 0


    if distance < 0:
        s = -(abs(distance) - abs(distance*0.1)) # reduce Distance to change angle(m)
    else: 
        s = (abs(distance) - abs(distance*0.1)) # reduce Distance to change angle(m)
    if abs(s) > abs(distance):
        s = 0 # this is to avoid the adjusted distance send the drone to the other side of the line
    print("Adjusted angle by reduce distance: ",s)
    #Define the ellipsoid
    geod = Geodesic.WGS84
    dir = geod.Direct(A[0],A[1],azi,s)
    C = (dir['lat2'],dir['lon2'])
    #print("Waypoint generated")
    return round(C[0],7),round(C[1],7),s

# **************************************************************************
# *******************************************     GPS LOG GENERATING *******                                      
# ************************************************************************** 
def createGPSLog():
    return True

# ***************************************************************************************************************************
# **************************************************************************     THREAD FUNCTIONS    ************************                                    
# *************************************************************************************************************************** 
#                                                                                                                          **
# This block contains threading function for parallel inspection                                                           **
#                                                                                                                          **
# ***************************************************************************************************************************
# ***************************************************************************************************************************
def navigationScript(waypoints=[]):
    """
    This function navigating the drone straight to waypoints
    """
    global heading, start_thread, end_mission, next_gimbal_target,turn_off,keep_direction
    i = 0
    while i<len(waypoints):
        if start_thread is True and turn_off == 0:
            if waypoints[i][0] == 'pylon_45' or waypoints[i][0] == 'pylon_135' or waypoints[i][0] == 'line'  : # only go to pylon 90 
                if i == 2: # first pylon 45
                    print("---------------------------------------------------------------------------------------")
                    print("Go to waypoint ",i," : ", waypoints[i])
                    print("---------------------------------------------------------------------------------------")
                    if keep_direction == 1:
                        goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],heading)
                    else:
                        goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4])
                    i+=1
                elif waypoints[i][0] == 'pylon_135' and waypoints[i+1][0] == 'rally' or  waypoints[i][0] == 'pylon_45' and waypoints[i+1][0] == 'rally': # last forward pylon 135 and last pylon 45
                    print("---------------------------------------------------------------------------------------")
                    print("LAST FORWARD PYLON 45")
                    print("Go to waypoint ",i," : ", waypoints[i])
                    print("---------------------------------------------------------------------------------------")
                    if keep_direction == 1:
                        goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],heading)
                    else:
                        goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4])
                    i+=1
                elif waypoints[i][0] == 'pylon_45' and waypoints[i-1][0] == 'rally' : # first turnback pylon 45
                    print("---------------------------------------------------------------------------------------")
                    print("LAST PYLON 45")
                    print("Go to waypoint ",i," : ", waypoints[i])
                    print("---------------------------------------------------------------------------------------")
                    if keep_direction == 1:
                        goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],heading)
                    else:
                        goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4])
                    i+=1
                else: # other pylons
                    i+=1
            else:
                print("---------------------------------------------------------------------------------------")
                print("Go to waypoint ",i," : ", waypoints[i])
                print("---------------------------------------------------------------------------------------")
                if keep_direction == 1:
                    goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],heading)
                elif i ==0:
                    goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4]) # first rally point, there is no i=-1
                else:
                    goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i-1][4])
                i+=1
        else:
            time.sleep(0.1)
            pass
    end_mission = True
    #print("END of script")
        
def photoCaptureScript(waypoints=[],gimbal_waypoints=[]):
    """
    This function point the gimbal and send capture commands for camera
    """
    global heading, total_capture, start_thread, curr_loc,turn_off,msg_update_rate,min_interval,max_interval
    next = False
    i = 2
    time_start = time.time()
    start_interval = time.time()
    while i<len(waypoints) and turn_off == 0:
        if start_thread is True:
            pos_reached = False
            while turn_off == 0:
                pos_reached,currentPos = checkPosReached(waypoints[i][1],waypoints[i][2],waypoints[i][3]) # check if the drone reached the inspection position
                next = pointCameraToward_v2(currentPos,(gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])) # check if we can point the camera to the pos, or it's a missed
                if next is True: # if the point is a missed
                    missed_list.append((i,waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4]))
                    i+=1
                    break # move to next target
                elif pos_reached is True: # else if we reach the right point
                    if i == int(len(waypoints)/2): # check ih we reach turn back rally points, spare time for the drone to change heading
                        time.sleep(3)
                        i+=1
                        break
                    print("POS ",i," REACHED:",pos_reached) # inform 
                    if time.time() - time_start > 0 and gimbal_waypoints[i][0] != 'rally' : # check if we got enough interval time
                        capturePhoto()
                        interval = time.time() - start_interval
                        print("Interval:",interval)
                        start_interval = time.time()
                        total_capture +=1
                        time_start = time.time()
                        i+=1
                        if interval > max_interval:
                            max_interval = interval
                        elif interval< min_interval:
                            min_interval = interval
                        break
                    else:
                        i+=1
                        break
                time.sleep(1/(msg_update_rate))

        else:
            time.sleep(1/msg_update_rate)
            pass

# ***************************************************************************************************************************
# **************************************************************************     MAIN  PROGRAM    ***************************                                    
# *************************************************************************************************************************** 
#                                                                                                                          **
# Main program to run in RTR_Powerline_inspection service                                                                  **
#                                                                                                                          **
# ***************************************************************************************************************************
# ***************************************************************************************************************************
if __name__=="__main__":

    # =====================================================================================
    # ========================================================================== PRE-FLIGHT
    # =====================================================================================
    print("================================================================================================")
    print("===================================================    PREFLIGHT: CHECK CONNECTION    ==========") 
    print("================================================================================================")
    checkConnection()
    # ================================== INIT UTILS ========================
    signal(SIGINT, handler)
    # ================================== INIT VARS =========================
    
    file_path = waypointFileConvert(file_path)

    # =====================================================================================
    # ======================================================================== MISSION LOOP
    # =====================================================================================
    # ================================== READ PYLONS LOC FILE ==============
    print("================================================================================================")
    print("===================================================    STEP 1: READ PYLON GPS FILE    ==========")
    print("================================================================================================")
    pylons = readGPS(file_path)
    #print(pylons)

    # ================================== GENERATE WAYPOINTS ================
    print("================================================================================================")
    print("===================================================    STEP 2: GENERATE WAYPOINTS    ===========")
    print("================================================================================================")
    # (inspection positions including lines ---- pylons GPS , copter inspection distance, yaw threshold
    inspection_points = generatePylonInspectionWaypoints(pylons,inspection_distance,0) 
    
    waypoints, gimbal_waypoints = generatePowerlineInspectionMission_v2\
        (pylons,inspection_points,line_point,inspection_distance)
    waypoints = sortWaypoints(waypoints)
    gimbal_waypoints = sortWaypoints(gimbal_waypoints)   

    # print(" ")
    # for waypoint in waypoints:
    #     print(waypoint)
    # print(" ")  
    # for gimbal_waypoint in gimbal_waypoints:
    #     print(gimbal_waypoint)
    # print(" ")

        # inspection point GPS, number of spline control point,inspection distance
    #waypoints = generatePylonInspectionWaypointsDebug(inspection_points)

    # print(" ")
    # print(inspection_points)
    # print(" ")
    # print(waypoints)
    # print(" ")
    # print(gimbal_waypoints)
    # print(" ")

    # ================================== PREFLIGHT CHECK AND INIT ==========
    print("================================================================================================")
    print("===================================================    STEP 3.1: PREFLIGHT    ==================")
    print("================================================================================================")

    total_photo = len(waypoints) - 8

    # ================================== ARM AND TAKE OFF ==================
    print("================================================================================================")
    print("=================================================    STEP 3.2: MISSION START !!!    ============")
    print("================================================================================================")
    
    print("Rotate camera to 0")
    gimbalRotate(0,0,0)
    thread_check_gimbal = Thread(target = checkGimbal,args = ()) # gimbal heartbeat send continously
    thread_check_gimbal.daemon = True 
    thread_check_gimbal.start() 
    print("0.INIT GIMBAL CHECK DONE")
    time.sleep(2)

    print("Capture test photo")
    capturePhoto() #test capture photo

    thread_check_mode = Thread(target = checkCurrentMode,args = ())
    thread_check_mode.daemon = True
    thread_check_mode.start()
    print("1.INIT CHECK FLIGHT MODE DONE")
    # ================================== CHANGE TO GUIDED ==================
    #changeMode('GUIDED')
    thread_get_pos = Thread(target = getCurrentPos ,args = ())
    thread_get_pos.daemon = True
    thread_get_pos.start()
    print("2.INIT GET POS DONE")

    print("Wait for GUIDED MODE to ARM")
    waitUntilGuided()
    armAndTakeoff(15)


    # ================================== SET MISSION HEADING ===============
    if keep_direction == 1:
        print("Heading change to:",heading) # keep heading
        setHeading(heading, False) # ------- false: relative frame or not
        time.sleep(1)
    else:
        print("Heading change to:",waypoints[2][4]) #heading to the first capture point
        setHeading(waypoints[2][4], False) # ------- false: relative frame or not
        time.sleep(1)

    # ================================== SET MISSION SPEED =================
    print("Set speed mission to: ",mission_speed)
    setSpeed(mission_speed)
    time.sleep(0.1)
    # ================================== SET POS MSG INTERVAL ==============
    #setMsgInterval() # set global pos rate to 0.01s


    thread_navigation = Thread(target = navigationScript ,args = (waypoints,))
    thread_navigation.daemon = True
    thread_navigation.start()
    print("3.INIT NAVIGATION DONE")
    thread_camera = Thread(target = photoCaptureScript ,args = (waypoints, gimbal_waypoints,))
    thread_camera.daemon = True
    thread_camera.start()
    print("4.INIT CAPTURING DONE")

    # ================================== CARRY ON MISSION ==================
    if inspection_mode == "dual":
        start_thread = False
        mission_start = time.time()
        i = 0
        while i < len(waypoints) and current_mode == 'GUIDED':

            if waypoints[i][0] == "pylon_90":
                print("---------------------------------------------------------------------------------------")
                print("Go to waypoint ",i," : ", waypoints[i][0])
                print("---------------------------------------------------------------------------------------")
                if keep_direction == 1:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptPylon(inspection_pos,heading,gimbal_pos) #lat lng alt heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon
                else:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptPylon(inspection_pos,waypoints[i][4],gimbal_pos) #lat lng alt waypoint_heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon

            elif waypoints[i][0] == "pylon_45":
                print("---------------------------------------------------------------------------------------")
                print("Go to waypoint ",i," : ", waypoints[i][0])
                print("---------------------------------------------------------------------------------------")
                if keep_direction == 1:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptPylon(inspection_pos,heading,gimbal_pos) #lat lng alt heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon
                else:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptPylon(inspection_pos,waypoints[i][4],gimbal_pos) #lat lng alt waypoint_heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon

            elif waypoints[i][0] == "pylon_135":
                print("---------------------------------------------------------------------------------------")
                print("Go to waypoint ",i," : ", waypoints[i][0])
                print("---------------------------------------------------------------------------------------")
                if keep_direction == 1:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptPylon(inspection_pos,heading,gimbal_pos) #lat lng alt heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon
                else:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptPylon(inspection_pos,waypoints[i][4],gimbal_pos) #lat lng alt waypoint_heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon

            elif waypoints[i][0] == "line":
                print("---------------------------------------------------------------------------------------")
                print("Go to waypoint ",i," : ", waypoints[i][0])
                print("---------------------------------------------------------------------------------------")
                if keep_direction == 1:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptLine(inspection_pos,heading,gimbal_pos) #lat lng alt heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon
                else:
                    inspection_pos = (waypoints[i][1],waypoints[i][2],waypoints[i][3])
                    gimbal_pos = (gimbal_waypoints[i][1],gimbal_waypoints[i][2],gimbal_waypoints[i][3])
                    missionScriptLine(inspection_pos,waypoints[i][4],gimbal_pos) #lat lng alt waypoint_heading gimbal_direction
                    i+=1  # add i+=1 here to check if we miss a pylon

            elif waypoints[i][0] == "rally":
                print("---------------------------------------------------------------------------------------")
                print("Go to waypoint ",i," : ", waypoints[i][0], "|| alt: ", waypoints[i][3])
                print("---------------------------------------------------------------------------------------")
                if keep_direction == 1:
                    goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],heading)
                    time.sleep(1)
                    i+=1  # add i+=1 here to check if we miss a pylon
                else:
                    goTo(waypoints[i][1],waypoints[i][2],waypoints[i][3],waypoints[i][4])
                    i+=1  # add i+=1 here to check if we miss a pylon
            else:
                print("WRONG WAYPOINT")
                break

            time.sleep(0.05)
  
        print("All waypoints reached. MISSION COMPLETED")
        RTL()
        #changeMode('RTL')
        time.sleep(5)
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        # ================================== RTL AND FINISH MISSION =============
        print("=========================================================")
        print("MISSION SUMMARY")
        print("TOTAL PYLONS:",len(pylons))
        print("TOTAL WAYPOINTS:",len(waypoints))
        print("TOTAL DISTANCE: ",round(total_distance,3)," meters")
        print("TOTAL CAPTURE:",total_capture)
        print("TOTAL TIME: ",round(time.time()-mission_start,3)," seconds")
        print("AVG SPEED: ",round(total_distance/(time.time()-mission_start),3)," m/s")
        print("MAX INTERRVAL: ",round(max_interval,3)," s")
        print("MIN INTERVAL: ",round(min_interval,3)," s")
        print("=========================================================")
        master.close()
    elif inspection_mode == "single":
        print("Mode not developed yet")
        master.close()
    elif inspection_mode == "pylon":
        print("Mode not developed yet")
        master.close()
    elif inspection_mode == "parallel":
        print("MISSION START IN PARALLEL MODE")
        mission_start = time.time()
        start_thread = True # init start mission
        i = 0
        while True:
            if current_mode != 'GUIDED' or end_mission is True:
                start_thread = False
                break
            time.sleep(0.1)

        print("All waypoints reached. MISSION COMPLETED")
        turn_off = 1
        RTL()
        #changeMode('RTL')
        time.sleep(5)
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        # ================================== RTL AND FINISH MISSION =============
        print("=========================================================")
        print("MISSION SUMMARY")
        print("TOTAL PYLONS:",len(pylons))
        print("TOTAL WAYPOINTS:",len(waypoints))
        print("TOTAL DISTANCE: ",round(total_distance,3)," meters")
        print("TOTAL PHOTO:",total_photo)
        print("TOTAL CAPTURED:",total_capture)
        print("TOTAL TIME: ",round(time.time()-mission_start,3)," seconds")
        print("AVG SPEED: ",round(total_distance/(time.time()-mission_start),3)," m/s")
        print("MAX INTERRVAL: ",round(max_interval,3)," s")
        print("MIN INTERVAL: ",round(min_interval,3)," s")
        print("MISSED: ",missed_waypoints)
        print("MISSED LIST:")
        for missed in missed_list:
            print(missed)
        print("=========================================================")
        master.close()