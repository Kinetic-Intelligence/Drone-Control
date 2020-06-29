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

TargeAltitude = 20
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


#---- MAIN FUNCTION
#- Takeoff
# Initialize the takeoff sequence to 20m
arm_and_takeoff(TargeAltitude)

print("Take off complete")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
print(vehicle.mode)
time.sleep(5)
vehicle.mode = VehicleMode("GUIDED_NOGPS")
print(vehicle.mode)
time.sleep(5)
vehicle.mode = VehicleMode("GUIDED_NOGPS")
print(vehicle.mode)
time.sleep(5)
# Hover for 10 seconds
vehicle.mode = VehicleMode("GUIDED_NOGPS")
print(vehicle.mode)
time.sleep(40)
vehicle.mode = VehicleMode("GUIDED")
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
