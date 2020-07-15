"""
Simple script for take off and controlling the yaw
"""

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#- Importing Tkinter: sudo apt-get install python-tk
import Tkinter as tk

#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')

TargeAltitude = 10
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

def land():
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
#---- MAIN FUNCTION
#- Takeoff
# Initialize the takeoff sequence to 20m
for x in range(10):
    print("TakeOff number %d "%(x+1))
    arm_and_takeoff(TargeAltitude)
    print("Take off complete")
    time.sleep(2)
    lock_gps()
    time.sleep(10)
    land()
    time.sleep(10)

# Close vehicle object
vehicle.close()
