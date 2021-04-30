import glob
import os
import sys

try:
    sys.path.append(glob.glob('C:/carla/CARLA_0.9.10/WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import random
import math
import numpy as np
import time
import transforms3d
import sklearn
from sklearn.cluster import KMeans
import threading
import matplotlib.pyplot as plt

actor_list = [] 
waypoint = carla.Location()
angle = 0
#Order = vehicle, spectator camera, sensor1, sensor2
lock = threading.Lock()

def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

      # Read the opendrive file to a string
    xodr_path = "speedway_5lanes.xodr"
    #xodr_path = "Crossing8Course.xodr"
    od_file = open(xodr_path)
    data = od_file.read()


    # Load the opendrive map
    vertex_distance = 2.0  # in meters
    max_road_length = 10.0 # in meters #Changed from 50.0 
    wall_height = 5.0      # in meters
    extra_width = 0.6      # in meters
    world = client.generate_opendrive_world(
        data, carla.OpendriveGenerationParameters(
        vertex_distance=vertex_distance,
        max_road_length=max_road_length,
        wall_height=wall_height,
        additional_width=extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True))
    
    spectator = world.get_spectator()

    spawn_actor(world)

    while True:
        #When all actors have been spawned
        if(len(actor_list) >= 4):
            control_vehicle(actor_list[0])
            camera = actor_list[1]
            spectator.set_transform(camera.get_transform())
        timestamp = world.wait_for_tick()
        delta_seconds = timestamp.delta_seconds
        time.sleep(delta_seconds)

def control_vehicle(vehicle):
    global angle
    print("angle = " + str(angle))
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    throttle = 0.75
    vehicle_transform = vehicle.get_transform()
    steer = degrees_to_steering_percentage(angle)
    print("steer = " + str(steer))
    control = carla.VehicleControl(throttle, steer)
    vehicle.apply_control(control)



def spawn_actor(world):
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    waypoints = world.get_map().generate_waypoints(10.0)

    targetLane = -3
    waypoint = waypoints[500]
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation

    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle)
    vehicle.set_simulate_physics(True)
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

    #Add spectator camera
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-30,0,0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera) #Add to actor_list at [1]

    attach_lidar(world, vehicle, transform)
    attach_collision_sensor(world, vehicle, transform)

    return vehicle, camera

def attach_lidar(world, vehicle, transform):
    #configure LIDAR sensor to only output 2d
    # Find the blueprint of the sensor.
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Set the time in seconds between sensor captures
    lidar_bp.set_attribute('sensor_tick', '0.1')
    lidar_bp.set_attribute('channels', '1')
    lidar_bp.set_attribute('upper_fov', '0')
    lidar_bp.set_attribute('lower_fov', '0')
    lidar_bp.set_attribute('range', '30') #10 is default

    lidar_bp.set_attribute('points_per_second', '500')
    #With 2 channels, and 100 points per second, here are 250 points per scan


    lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    actor_list.append(lidar_sensor) #Add at actor_list[2]

    #Commented out to decrease processing
    lidar_sensor.listen(lambda data: save_lidar_image(data, world, vehicle))

    return lidar_sensor



def attach_collision_sensor(world, vehicle, transform):
    #Configure collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=vehicle)
    actor_list.append(collision_sensor) #Add at actor_list[3]

    collision_sensor.listen(lambda data: collision_event(data, world, vehicle))

    return collision_sensor


def collision_event(data, world, vehicle):
    print("COLLISION")

    a = len(actor_list)
    for b in range(a):
           actor_list[a-1].destroy()
           actor_list.pop(a-1)
           print(actor_list)
           a = a-1

    spawn_actor(world)
    time.sleep(1)

def polar_to_cartesian(polar_coordinates):
    cartesian_coordinates = []
    for point in polar_coordinates:
        r = point[0] 
        theta = point[1]

        x = r * math.cos(theta * math.pi/180.0)
        y = r * math.sin(theta * math.pi/180.0)

        cartesian_point = [x, y, point[2]]
        cartesian_coordinates.append(cartesian_point)
    return cartesian_coordinates

def cartesian_to_polar(cartesian_coordinates):
    #Parameter: an array of 3d coordinate triplets
    # [[X1, Y1, Z1], [X2, Y2, Z2], ...]
    polar_coordinates = []
    
    for point in cartesian_coordinates:
        x = point[0] 
        y = point[1]
        radius = np.sqrt(x * x + y * y)
        theta = np.arctan2(y,x)
        theta = 180 * theta / math.pi #Convert from radians to degrees
        polar_point = [radius, theta, point[2]]
        polar_coordinates.append(polar_point)
    return polar_coordinates

def graph_polars(polar_coordinates):

    time.sleep(1)

    polar_coordinates.sort(key = lambda point: point[1])
    w = 4
    h = 3
    d = 70
    plt.figure(figsize=(w, h), dpi=d)
    thetas = np.array(polar_coordinates)[:,1] / 180 * math.pi
    r = np.array(polar_coordinates)[:,0]
    ax = plt.subplot(projection='polar')
    ax.plot(thetas, r, 'o', color='black')
    plt.savefig("mygraph.png")

    
def save_lidar_image(image, world, vehicle):
    global angle
    global lock

    if not lock.acquire(False):
        return

    #Convert raw data to coordinates (x,y,z)
    #print("SCAN:")
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    #print(image.horizontal_angle)
    #print(points.shape)
    points = np.reshape(points, (int(points.shape[0] / 3), 3))

    polars = cartesian_to_polar(points)
    
    graph_polars(polars)

    points = polar_to_cartesian(polars)

    polars.sort(key = lambda point: point[1])

    find_disparity(polars)

    # #reshape from list of 4d points to an array of 3d points 
    # transform = vehicle.get_transform()
    # transform = [transform.location.x, transform.location.y, transform.location.z]

    # #Rotate into the car's frame
    # vehicle_rotation = vehicle.get_transform().rotation
    # roll = vehicle_rotation.roll 
    # pitch = vehicle_rotation.pitch
    # yaw = vehicle_rotation.yaw + (np.pi / 2)
    # R = transforms3d.euler.euler2mat(roll,pitch,yaw).T

    # points = points.copy()

    # points[:] = [np.dot(R, point) for point in points]
    # points[:] = [np.add(transform, point)  for point in points]       #Move location into car's frame

    for point in points:
        location = carla.Location(x=float(point[0]), y=float(point[1]), z=float(0))
        world.debug.draw_point(location, life_time=1, color = carla.Color(0, 255, 255))


    lock.release()


def find_disparity(polar_coordinates):
    global angle
    max_distance = 0
    max_disparity_pair = [[],[]]

    polar_coordinates = [i for i in polar_coordinates if i[1] > 0]

    for i in range(len(polar_coordinates) - 1):
        r1 = polar_coordinates[i][0]
        theta1 =  polar_coordinates[i][1] / 180 * math.pi
        r2 = polar_coordinates[i+1][0]
        theta2 =  polar_coordinates[i+1][1] / 180 * math.pi
        distance = math.sqrt(abs(r1*r1 + r2*r2 - 2 *r1 *r2 * np.cos(theta1 - theta2)))
        if distance > max_distance:
            max_distance = distance
            max_disparity_pair[0] = polar_coordinates[i]
            max_disparity_pair[1] = polar_coordinates[i+1]

    angle = (max_disparity_pair[1][1] + max_disparity_pair[0][1]) /2 

    return 

def degrees_to_steering_percentage(degrees):
    """ Returns a steering "percentage" value between 0.0 (left) and 1.0
    (right) that is as close as possible to the requested degrees. The car's
    wheels can't turn more than max_angle in either direction. """
    degrees = -(degrees - 90)
    print("degrees = " + str(degrees))
    max_angle = 45
    if degrees < -max_angle:
        return 1.0
    if degrees > max_angle:
        return -1.0
    if abs(degrees) < 1:
        return 0
        
    return - (degrees / max_angle)


def get_right_lane_nth(waypoint, n):
    out_waypoint = waypoint
    for i in range(n):
        out_waypoint = out_waypoint.get_right_lane()
    return out_waypoint

def get_left_lane_nth(waypoint, n):
    out_waypoint = waypoint
    for i in range(n):
        out_waypoint = out_waypoint.get_left_lane()
    return out_waypoint

def change_lane(waypoint, n):
    if (n > 0):
        return get_left_lane_nth(waypoint, n)
    else:
        return get_right_lane_nth(waypoint, n)

def relative_location(frame, location):
    origin = frame.location
    forward = frame.get_forward_vector()
    right = frame.get_right_vector()
    up = frame.get_up_vector()
    disp = location - origin
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
    return carla.Vector3D(x, y, z)

def get_transform(vehicle_location, angle, d=6.4):
        a = math.radians(angle)
        location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
        return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('destroying actors')
        for actor in actor_list:
           actor.destroy()
        print('\ndone.')