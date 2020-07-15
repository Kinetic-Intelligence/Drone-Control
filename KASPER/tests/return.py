"""
Simple script for take off and controlling the yaw
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from threading import Thread, Event
import time
#- Importing Tkinter: sudo apt-get install python-tk
import Tkinter as tk

#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')

#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]

#---setup the yaw yaw_rate
yaw_rate = 3 #[rad/sec]

yaw_rad = 0.9730
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000 0010 0000 0000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
def condition_yaw(heading, relative=False):
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
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
#-- Define the function for sending mavlink velocity command in body frame
def set_coordinate_body(vehicle, x, y, z):

   msg = vehicle.message_factory.set_position_target_local_ned_encode(
           0,
           0, 0,
           mavutil.mavlink.MAV_FRAME_BODY_NED,
           0b0000100111111000, #-- BITMASK -> Consider only the position
           x, y, z,        #-- POSITION
           0, 0, 0,     #-- VELOCITY
           0, 0, 0,        #-- ACCELERATIONS
           0, 0)
   vehicle.send_mavlink(msg)
   vehicle.flush()
#-- Key event function
def key(cmd):
    if cmd == "land":
        print(vehicle.mode)
        print("Now let's land")
        vehicle.mode = VehicleMode("RTL")
        print("Coming back Down")
        while True:
           v_alt = vehicle.location.global_relative_frame.alt
           print(">> Altitude = %.1f m"%v_alt)
           if v_alt <= 0:
               print("On the ground")
               break
           time.sleep(1)
        # Close vehicle object
        vehicle.close()
    elif cmd == "south":
        print("Yaw 180 absolute (South)")
        condition_yaw(90)
    elif cmd == "east":
        print("Yaw 270 absolute (East)")
        condition_yaw(210)
    else:
        print("Unknown command")

def log_print():
    print "Autopilot Firmware version: %s" % vehicle.version
    print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
    print "Global Location: %s" % vehicle.location.global_frame
    print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print "Local Location: %s" % vehicle.location.local_frame    #NED
    print "Attitude: %s" % vehicle.attitude
    print "Velocity: %s" % vehicle.velocity
    print "GPS: %s" % vehicle.gps_0
    print "Groundspeed: %s" % vehicle.groundspeed
    print "Airspeed: %s" % vehicle.airspeed
    print "Gimbal status: %s" % vehicle.gimbal
    print "Battery: %s" % vehicle.battery
    print "EKF OK?: %s" % vehicle.ekf_ok
    print "Last Heartbeat: %s" % vehicle.last_heartbeat
    print "Rangefinder: %s" % vehicle.rangefinder
    print "Rangefinder distance: %s" % vehicle.rangefinder.distance
    print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print "Heading: %s" % vehicle.heading
    print "Is Armable?: %s" % vehicle.is_armable
    print "System status: %s" % vehicle.system_status.state
    print "Mode: %s" % vehicle.mode.name    # settable
    print "Armed: %s" % vehicle.armed    # settable

def lock_gps():
    print("Locking current GPS location")

    gps = str(vehicle.location.global_frame)
    print("%s" % gps)
    mysplit1 = gps.split(",")
    mysplit2_lat = mysplit1[0].split("=")
    mysplit2_log = mysplit1[1].split("=")
    lat = float(mysplit2_lat[1])
    log = float(mysplit2_log[1])

    point1 = LocationGlobalRelative(lat, log, 10)
    vehicle.simple_goto(point1)
    print("GPS locked")


#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)
print("Take off complete")
lock_gps()

#set_coordinate_body(vehicle, 0, 10, 0)
#time.sleep(10)
#- Read the keyboard with tkinter

print("Enter Command to controll kite")

timeout = 10
def gfg():
    for x in range(10):
        command = raw_input(">>")
        if command != "":
            key(command)
        command = ""

# Event object used to send signals from one thread to another
stop_event = Event()

time.sleep(10)


if __name__ == '__main__':
    # We create another Thread
    action_thread = Thread(target=gfg)

    # Here we start the thread and we wait 5 seconds before the code continues to execute.
    action_thread.start()
    action_thread.join(timeout=60)

    # We send a signal that the other thread should stop.
    stop_event.set()

    print("Timeout landing the dronekite")


if vehicle.mode != VehicleMode("RTL"):
    print(vehicle.mode)
    print("Now let's land")
    vehicle.mode = VehicleMode("RTL")
    print("Coming back Down")
    while True:
       v_alt = vehicle.location.global_relative_frame.alt
       print(">> Altitude = %.1f m"%v_alt)
       if v_alt <= 0:
           print("On the ground")
           break
       time.sleep(1)
    # Close vehicle object
    vehicle.close()
