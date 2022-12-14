import math
import time
from pymavlink import mavutil
import sys

def connectToVehicle(ipaddress):
    #establishes the connection to our vehicles ip_address from the onboard computer or rpi
    conn = mavutil.mavlink_connection("udp"+ipaddress+":5760") 

    #sets the system and component ID for the remote system
    conn.wait_heartbeat()
    #when heartbeat is recieved target system and target component values are populated
    print("System Heartbeat (system %u component %u)"%(conn.target_system, conn.target_component))

    # Get some information on the connection to check if established or not !
    while True:
        try:
            print(conn.recv_match().to_dict())
            return conn
        except:
           time.sleep(1) #sleep for one second


def armDrone(conn):
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0)
    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    conn.motors_armed_wait()
    print('Armed!')


def disarmDrone(conn):
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0)
    # wait until disarming confirmed
    conn.motors_disarmed_wait()


def takeOff(conn,height_meters):
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF ,0,0,0,0,0,0,height_meters)


def landing(conn):
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,0,"PRECISION_LAND_MODE",0,0,0,0,0)


def moveToLocation(conn,longitude,latitude,altitude,radiusDistance):
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,radiusDistance,0,0,latitude,longitude,altitude)

#command acknowledge tells us the result of success/failure/in-progress and include information about failure reasons or progress information
def recieveMessage(conn):
    msg= conn.recv_match(type="COMMAND_ACK",blocking=True)
    print(msg)


# 0	MAV_MODE_PREFLIGHT	System is not ready to fly, booting, calibrating, etc. No flag is set.
# 80	MAV_MODE_STABILIZE_DISARMED	System is allowed to be active, under assisted RC control.
# 208	MAV_MODE_STABILIZE_ARMED	System is allowed to be active, under assisted RC control.
# 64	MAV_MODE_MANUAL_DISARMED	System is allowed to be active, under manual (RC) control, no stabilization
# 192	MAV_MODE_MANUAL_ARMED	System is allowed to be active, under manual (RC) control, no stabilization
# 88	MAV_MODE_GUIDED_DISARMED	System is allowed to be active, under autonomous control, manual setpoint
# 216	MAV_MODE_GUIDED_ARMED	System is allowed to be active, under autonomous control, manual setpoint
# 92	MAV_MODE_AUTO_DISARMED	System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
# 220	MAV_MODE_AUTO_ARMED	System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
# 66	MAV_MODE_TEST_DISARMED	UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
# 194	MAV_MODE_TEST_ARMED	UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
def changeFlightMode(conn,flightMode):
    #below section checks if mode we chose is a valid option
    if flightMode not in conn.mode_mapping():
        print('Unknown mode : {}'.format(flightMode))
        print('Try:', list(conn.mode_mapping().keys()))
        sys.exit(1)

    # MODE_MAPPING retrieves the mode_id for the chosen flight mode
    mode_id = conn.mode_mapping()[flightMode]
    # this Sets the new mode with MAV_CMD_NAV_SET_MODE and modeID as the 6th parameter
    conn.mav.command_long_send(conn.target_system, conn.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,0, mode_id, 0, 0, 0, 0, 0)    

    # The COMMAND_ACK is used to retrieve data on the command we initiated and returns if the command was initiated,waiting or encountered error
    while True:
        ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

    #Polls until the command is set and acknowedged
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

    #Retrieves information on the status of the above command
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

#this command will either opne or close a gripper device used to hold payload, True opens gripper and False closes gripper
def openGripper(conn,open): 
    if open==True:
        conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_DO_GRIPPER	,1,0,0,0,0,0,0)
    else:
        conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_DO_GRIPPER	,1,1,0,0,0,0,0)

#Helper method to calculate the distance between two longitude and latitude coordinates
def getDistanceToLocationInMeters(lat1,lon1,lat2,lon2):
    earthRadius = 6371 
    dLat = degreeToRadians(lat2-lat1)  
    dLon = degreeToRadians(lon2-lon1) 
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(degreeToRadians(lat1)) * math.cos(degreeToRadians(lat2)) * math.sin(dLon/2) * math.sin(dLon/2)
    
    b = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a)) 
    c = (earthRadius * b)*1000; # Distance in meters
    return c;

def degreeToRadians(deg):
    return deg * (math.PI/180)

def setSpeed(conn,speed):
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SPEED,0,speed,0,0,0,0,0)

def mission(ipaddress,longitude,latitude,altitude,radiusDistance):
    #connect to vehicle
    conn= connectToVehicle(ipaddress)

    #change to pre flight mode
    conn.mav.command_long_send(conn.target_system, conn.target_component,mavutil.mavlink.MAV_DO_SET_MODE,MAV_MODE_PREFLIGHT,0,0,0, 0, 0, 0) 
    time.sleep(2)#allow two seconds for command to be recieved
    #recieve data on changing to pre flight mode

    while True:
        ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break
    

    #pre flight calibration for gyroscope, magnetometer, accelerometer,esc
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,1,1,0,0,1,0,1)
   
    #Arm drone motors
    armDrone(conn)

    #switch to GUIDED mode to engage takeoff
    while conn.mode!="MAV_MODE_GUIDED_ARMED":
        changeFlightMode("MAV_MODE_GUIDED_ARMED")
        time.sleep(1)
        
    
    openGripper(conn,False) #close Gripper to hold payload
    
    takeOff(conn,10) #attempt take off to 10m height
    time.sleep(2)

    #Recieves information on Takeoff command status
    while True:
        ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            continue
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    #Check we have reached at least 95% of target altitude
    while conn.location.global_relative_frame.alt <= .95*10:
        print("Current Altitude: %d" %conn.location.global_relative_frame.alt)

    distance= getDistanceToLocationInMeters(conn.location.global_relative_frame.lon,conn.location.global_relative_frame.lat ,longitude,latitude)
    timeToArrival= distance/2 #calculate time to arrival by dividing meters/(meters per second)
    setSpeed(conn,2)#set speed to 2 meters per second

    moveToLocation(conn,longitude,latitude,altitude,radiusDistance)
    time.sleep(2)

    #Recieve data on navigate to waypoint command
    while True:
        ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            continue
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    time.sleep(timeToArrival) #sleep until arrival to waypoint

    landing(conn) #engage landing procedure

    #Check we have reached close to the ground level for our position
    while conn.location.global_relative_frame.alt >= .1:
        print("Current Altitude: %d" %conn.location.global_relative_frame.alt)
    

    openGripper(conn,True)# release payload
    time.sleep(4) #allows time for payload release before initiating next command
    
    #takeoff to ten meters again 
    takeOff(conn,10)
    time.sleep(2)

    #retrieve information on takeoff command
    while True:
        ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            continue
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    #wait until we reach takeoff height of 10 meters 
    while conn.location.global_relative_frame.alt <= .95*10:
        print("Current Altitude: %d" %conn.location.global_relative_frame.alt)

    #command to return to home position
    conn.mav.command_long_send(conn.target_system,conn.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0)

    #recieve message on Return to launch command
    while True:
        ack_msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            continue
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    #sleep for time it takes to return back to home location
    time.sleep(timeToArrival)

    #engage landing
    landing(conn)

    #check we have approached ground level
    while conn.location.global_relative_frame.alt >= .1:
        print("Current Altitude: %d" %conn.location.global_relative_frame.alt)

    #close connection
    conn.close()

#run mission by calling above method we made
mission() #add params to specify where we wish to go for the mission, Currently no destination information as we have not chosen where to go.
