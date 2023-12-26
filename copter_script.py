from dronekit import (
    connect,
    VehicleMode,
    LocationGlobalRelative,
    LocationGlobal,
)
import time
import math
from pymavlink import mavutil

# Connect to Mission Planner SITL
vehicle = connect("127.0.0.1:14551", wait_ready=True)

point_a = LocationGlobal(50.450739, 30.461242, 0.0)  # altitude to 0 (sea level)
point_b = LocationGlobalRelative(50.443326, 30.448078, 100)
target_yaw = 350


def condition_yaw(heading, relative=False) -> None:
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        1,
        is_relative,
        0,
        0,
        0,
    )
    # send command to vehicle
    print("making rotate...")
    vehicle.send_mavlink(msg)
    vehicle.flush()


def naw_waypoint(point) -> None:
    """
    :param point: LocationGlobalRelative object
    :return None:
    """
    msg = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        0,
        0,
        0,
        0,
        point.lat,
        point.lon,
        point.alt,
    )
    # send command to vehicle
    print("flyng to destanation...")
    vehicle.send_mavlink(msg)
    vehicle.flush()


def get_distance_metres(aLocation1, aLocation2) -> int:
    """
    :param aLocation1: LocationGlobalRelative1
    :param aLocation2: LocationGlobalRelative2

    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.floor(math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5)


def arm_and_takeoff(aTargetAltitude) -> None:
    """
    :param aTargetAltitude: int
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if (
            vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95
        ):  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


print("Starting mission")

arm_and_takeoff(10)


vehicle.mode = VehicleMode("AUTO")
naw_waypoint(point_b)
while True:
    distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)
    print(f"Distance to waypoint : {distance_to_point}")
    if distance_to_point == 0:
        print("Reached target location")
        break
    time.sleep(1)

vehicle.mode = VehicleMode("STABILIZE")

vehicle.mode = VehicleMode("GUIDED")
condition_yaw(target_yaw)
time.sleep(10)
vehicle.mode = VehicleMode("STABILIZE")

# print('Return to launch')
# vehicle.mode = VehicleMode("RTL")


# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
