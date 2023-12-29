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

point_from = [50.450739, 30.461242]
point_to = [50.443326, 30.448078]  # [ 50.443326, 30.448078 ]  #

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

    vehicle.mode = VehicleMode("ALT_HOLD")
    fly_program()


SLOW_DISTANCE = 50


def get_axis(desired_radiant, current_radiant):
    delta = desired_radiant - current_radiant
    if delta > math.pi:
        delta = delta - math.pi
    if abs(delta) < 0.1:
        return 0
    else:
        print(delta)
        rotation_speed = delta / math.pi * 100
        if abs(rotation_speed) < 50:
            rotation_speed = -50 if rotation_speed < 0 else 50
        return rotation_speed


def get_pitch_axis(current_dist):
    if current_dist < SLOW_DISTANCE:
        return 100
    else:
        return 500


def calculate_bearing(point_a, point_b):
    # Convert latitude and longitude to radians
    lat1, lon1 = math.radians(point_a[0]), math.radians(point_a[1])
    lat2, lon2 = math.radians(point_b[0]), math.radians(point_b[1])

    # Calculate differences in longitude
    delta_lon = lon2 - lon1

    # Calculate bearing in radians
    bearing_radians = math.atan2(
        math.sin(delta_lon) * math.cos(lat2),
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon),
    )

    return bearing_radians


def make_rotation_to_point(new_desired_radiant_yaw):
    current_radiant_yaw = vehicle.attitude.yaw
    rotation_speed = get_axis(new_desired_radiant_yaw, current_radiant_yaw)
    if rotation_speed != 0:
        vehicle.channels.overrides = {
            "1": int(1500),  # roll
            "2": int(1500),  # pitch
            "3": int(1500),  # throttle
            "4": int(1500 + rotation_speed),  # yaw
        }
        time.sleep(0.1)


def reset_rotation_to_point():
    current_point = [
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
    ]

    desired_radiant_yaw = calculate_bearing(current_point, point_to)
    current_radiant_yaw = vehicle.attitude.yaw

    max_iterations = 100
    tolerance = 10.0  # Set your tolerance value here
    iterations = 0
    while (
        abs(math.degrees(desired_radiant_yaw) - math.degrees(current_radiant_yaw))
        > tolerance
        and iterations < max_iterations
    ):
        current_radiant_yaw = vehicle.attitude.yaw
        rotation_speed = get_axis(desired_radiant_yaw, current_radiant_yaw)
        vehicle.channels.overrides = {
            "1": int(1500),  # roll
            "2": int(1500),  # pitch
            "3": int(1500),  # throttle
            "4": int(1500 + rotation_speed),  # yaw
        }
        time.sleep(0.1)
        iterations += 1


def reset_rotation_to_point_precise():
    current_point = [
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
    ]

    desired_radiant_yaw = calculate_bearing(current_point, point_to)
    current_radiant_yaw = vehicle.attitude.yaw

    max_iterations = 1000
    tolerance = 1.0  # Set your tolerance value here
    iterations = 0
    while (
        abs(math.degrees(desired_radiant_yaw) - math.degrees(current_radiant_yaw))
        > tolerance
        and iterations < max_iterations
    ):
        current_radiant_yaw = vehicle.attitude.yaw
        rotation_speed = get_axis(desired_radiant_yaw, current_radiant_yaw)
        vehicle.channels.overrides = {
            "1": int(1500),  # roll
            "2": int(1500),  # pitch
            "3": int(1500),  # throttle
            "4": int(1500 + rotation_speed),  # yaw
        }
        time.sleep(0.1)
        iterations += 1


def fly_program():
    reset_rotation_to_point()

    distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)

    dist_tolerance = 1

    while distance_to_point >= 0 + dist_tolerance:

        distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)
        current_point = [
            vehicle.location.global_frame.lat,
            vehicle.location.global_frame.lon,
        ]

        new_desired_radiant_yaw = calculate_bearing(current_point, point_to)
        current_radiant_yaw = vehicle.attitude.yaw

        distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)
        vehicle.channels.overrides = {
            "1": int(1500),  # roll
            "2": int(1500 - get_pitch_axis(distance_to_point)),  # pitch
            "3": int(1500),  # throttle
            "4": int(1500),  # yaw
        }
        print(f"Distance to waypoint : {distance_to_point}")
        time.sleep(0.1)

        tolerance = 1.0  # Set your tolerance value here
        if (
            abs(
                math.degrees(new_desired_radiant_yaw)
                - math.degrees(current_radiant_yaw)
            )
            > tolerance
        ):  # optimal tolerance
            make_rotation_to_point(new_desired_radiant_yaw)

        distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)

        if distance_to_point < 15:
            while int(vehicle.groundspeed) != 0:
                vehicle.channels.overrides = {
                    "1": int(1500),  # roll
                    "2": int(1500 + get_pitch_axis(distance_to_point)),  # pitch
                    "3": int(1500),  # throttle
                    "4": int(1500),  # yaw
                }
            break

    vehicle.channels.overrides = {}

    print(f"Distance to waypoint after reset : {distance_to_point}")
    time.sleep(1)
    reset_rotation_to_point_precise()
    time.sleep(1)

    distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)
    while distance_to_point > 0 + dist_tolerance:
        vehicle.channels.overrides = {
            "1": int(1500),  # roll
            "2": int(1500 - get_pitch_axis(distance_to_point)),  # pitch
            "3": int(1500),  # throttle
            "4": int(1500),  # yaw
        }
        distance_to_point = get_distance_metres(vehicle.location.global_frame, point_b)
        print(f"Distance to waypoint percise : {distance_to_point}")
        time.sleep(0.1)

    time.sleep(1)

    # Reset all channel overrides
    vehicle.channels.overrides = {}


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

arm_and_takeoff(100)
time.sleep(2)

naw_waypoint(point_b)

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
