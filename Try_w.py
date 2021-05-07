# The codes included in Wolverine_CARLA_Thunderhill project are licensed under the terms of the MIT license, reproduced below.

# = = = = =

# The MIT License

# Copyright (c) 2015-2019 Jukka Lehtosalo and contributors

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# = = = = =
import sys
import glob
import os
import math
import csv
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

try:
    sys.path.append(glob.glob('C:/carla/CARLA_0.9.10/WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
try:
    sys.path += ['C:/carla/CARLA_0.9.10/WindowsNoEditor/PythonAPI/examples']
except IndexError:
    pass

    sys.path.append(glob.glob(''))

import matplotlib.pyplot as plt
import numpy as np
import time
# import pygame
import pandas as pd
import argparse
import os
import logging
import time
from datetime import datetime

import carla
from manual_control import (World, HUD, KeyboardControl, CameraManager,
                                     CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor)
import srunner	
from srunner.challenge.utils.route_manipulation import interpolate_trajectory

# Controller Class
import controller2d

global world

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    args = argparser.parse_args()

    args.rolename = 'hero'      # Needed for CARLA version
    args.filter = "vehicle.*"   # Needed for CARLA version
    args.gamma = 2.2   # Needed for CARLA version
    args.width, args.height = [int(x) for x in args.res.split('x')]
    ##Modifiable Variables from -1 to -5, as left to right
    # targetLane = -3
    targetLane = -3

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    # Read the opendrive file to a string

    # xodr_path = "speedway_5lanes.xodr"
    xodr_path = "Thunderhilll.xodr"
    od_file = open(xodr_path)
    data = od_file.read()

    # # Load the opendrive map
    vertex_distance = 2.0  # in meters
    max_road_length = 50.0 # in meters  10
    wall_height = 0.5     # in meters  1.0
    extra_width = 0.6      # in meters
    global waypoint_grid
    waypoint_grid = 5
    # global waypoint_search_grid
    # waypoint_search_grid = 10
    throttle_value = 0.9
    initialized_point = 0
    constant_speed = 10 # m/s
    global section
    section = 500

    world = client.generate_opendrive_world(
        data, carla.OpendriveGenerationParameters(
        vertex_distance=vertex_distance,
        max_road_length=max_road_length,
        wall_height=wall_height,
        additional_width=extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True))
    # world = client.load_world('Town02')
    # Automatic change sun angle
    now = datetime.now()
    current_hours = int(now.strftime('%H'))
    # Draw the plot below, you'll understand.
    # time = [ 0.  1.  2.  3.  4.  5.  6.  7.  8.  9. 10. 11. 12. 13. 14. 15. 16. 17. 18. 19. 20. 21. 22. 23.]
    # angle = [-90. -75. -60. -45. -30. -15.   0.  15.  30.  45.  60.  75.  90.  75. 60.  45.  30.  15.  0. -15. -30. -45. -60. -75.]
    # plt.plot(time,angle)
    sun_angle =  current_hours*(15)-90 if (current_hours <= 12 and current_hours >= 0) else current_hours*(-15)+270

    weather = carla.WeatherParameters(
    cloudiness=0.0,
    precipitation=0.0,
    sun_altitude_angle=sun_angle)
    world.set_weather(weather)
    spectator = world.get_spectator()

    map = world.get_map()
    # clock = pygame.time.Clock()
    count = 0


    # # Generate waypoints every 40 meters.
    # #   generate_waypoints(self, distance)
    # #   Returns a list of waypoints with a certain distance between them for every lane and centered inside of it. 
    # #   Waypoints are not listed in any particular order. 
    # #   Remember that waypoints closer than 2cm within the same road, section and lane will have the same identificator.

    # waypoint_list = map.generate_waypoints(waypoint_grid)
    # for w in waypoint_list:
    #     world.debug.draw_string(w.transform.location, 'x', draw_shadow=False,
    #                                 color=carla.Color(r=0, g=0, b=255, a =20), life_time=120.0,
    #                                 persistent_lines=True)

    # initial_counts = len(waypoint_list)
    # ## waypoint_list = map.generate_waypoints(0.5)
    
    # #Take only the waypoints from the targetLane
    # waypoints = single_lane(waypoint_list, targetLane)
    # # waypoint_list = waypoint_list
    # waypoint_list = waypoints
    # # waypoint_list = single_lane(waypoint_list, -1)
    # for w in waypoint_list:
    #     world.debug.draw_string(w.transform.location, 'O', draw_shadow=False,
    #                                 color=carla.Color(r=255, g=255, b=0, a =200), life_time=120.0,
    #                                 persistent_lines=True)
    # print("Length: " + str(len(waypoint_list)))

    # # Waypoints counts validation, since the manipulation of waypoints will crop some of them, 
    # # which will force the vehicle loose target
    # processed_counts = len(waypoint_list)
    # if initial_counts == processed_counts:
    #     print("Waypoints generated succeed!")
    #     print("Initial:", initial_counts, "Processed: ", processed_counts)
    # else:
    #     print('Not equal!!!!!')
    #     print("Initial:", initial_counts, "Processed: ", processed_counts)

    transform = carla.Transform(carla.Location(x=13, y=-0.01, z=0), carla.Rotation(pitch=0,yaw=180,roll=0))
    start_waypoint = map.get_waypoint(transform.location)
    # end_waypoint = start_waypoint.next(4556)[-1]
    end_waypoint = map.get_waypoint(transform.location + carla.Vector3D(0, -100, 0))

    waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
    gps_route, trajectory = interpolate_trajectory(world,waypoints,0.8)
    # print(gps_route)
    print(len(gps_route))
    print("**************************************")
    # print(trajectory)
    print(len(trajectory))

    #print(_get_waypoints(trajectory))
    waypoint_list = _get_waypoints(trajectory)
    waypoint_list = packWaypoints(waypoint_list[:section],world)
    for w in waypoint_list:
        world.debug.draw_string(w.transform.location, 'x', draw_shadow=False,
                                    color=carla.Color(r=0, g=0, b=255, a =20), life_time=120.0,
                                    persistent_lines=True)

    print("done")
    # filename = 'C:\Shengjie\carla_indy\T01_waypoint.txt'
    # text_save(filename, waypoints_list)
    # draw_trajectory(trajectory,200,0.5)

    #Remove all unneccesary waypoints along the straights
    curvy_waypoints =  waypoint_list
    # curvy_waypoints = waypoints
    # curvy_waypoints = get_curvy_waypoints(waypoints)

    # #Save graph of plotted points as bezier.png
    x = [p.transform.location.x for p in curvy_waypoints]
    y = [p.transform.location.y for p in curvy_waypoints]
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    plt.gca().invert_yaxis()
    plt.plot(x, y, marker = '.', markersize=1)
    plt.savefig("Path.png")

    #Set spawning location as initial waypoint
    waypoint = curvy_waypoints[initialized_point]


    # blueprint = world.get_blueprint_library().filter('vehicle.*tt*')[0]
    blueprint = world.get_blueprint_library().filter('vehicle.*mkz2017*')[0]

    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation
    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    print("SPAWNED!")
    
    #Vehicle properties setup
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)

    vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.All | carla.VehicleLightState.LowBeam | carla.VehicleLightState.HighBeam | carla.VehicleLightState.Interior))
    # vehicle.enable_constant_velocity(carla.Vector3D(constant_speed/math.sqrt(2),constant_speed/math.sqrt(2),0))

    # carla.command.SetVehicleLightState(0,carla.VehicleLightState.Interior)
    #Add spectator camera to get the view to move with the car 
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-20,0,0))
    camera_transform = carla.Transform(carla.Location(x=-10,z=7), carla.Rotation(-20,0,0))

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    # START
    time1 = time.time()
    ##INSERT MODIFYING WAYPOINTS HERE

    while True:
        # time_1 = carla.timestamp()
        time2 = time.time()
        dt = float("{0:.1f}".format(time2 - time1))
        #Update the camera view
        spectator.set_transform(camera.get_transform())

        #Get next waypoint
        # waypoint = get_next_waypoint(world, vehicle, curvy_waypoints)
        waypoint = getNexWaypoint(world, vehicle,curvy_waypoints)
        # waypoint = get_nearest_waypoints(vehicle,map)
        x_ = waypoint.transform.location.x
        y_ = waypoint.transform.location.y

        # world.debug.draw_point(waypoint.transform.location,size=0.1,color=(0,0,0,125),life_time=5)
        world.debug.draw_string(waypoint.transform.location, 'o', draw_shadow=False,
                                            color=carla.Color(r=0, g=255, b=0, a = 50), life_time=0.001,
                                            persistent_lines=True)

        #Control vehicle's throttle and steering
        throttle = throttle_value
        # throttle = 1.0
        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_x = round(vehicle_location.x,2)
        vehicle_y = round(vehicle_location.y,2)
        # print(dt)
        # if(dt%5 == 0.1):
            # print("Target: ", x_,y_,"Location: ",vehicle_x,vehicle_y)
        steer = control_pure_pursuit(vehicle_transform, waypoint.transform, max_steer, wheelbase)
        control = carla.VehicleControl(throttle, steer)
        vehicle.apply_control(control)
        world.debug.draw_string(vehicle_transform.location, str('%15.0f km/h' % (3.6*math.sqrt(vehicle.get_velocity().x**2+vehicle.get_velocity().y**2))), color=carla.Color(r=0, g=255, b=200, a = 50), life_time=0.0001,
                                            persistent_lines=True)
        # time_2 = carla.timestamp()
        world.debug.draw_string(vehicle_transform.location + carla.Location(x=1, y=0), str('%15.0f FPS' % (30)), color=carla.Color(r=100, g=0, b=10, a = 50), life_time=0.0001, persistent_lines=True)
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)
    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)

def test_only():
    """Main function.

    Args:
        -v, --verbose: print debug information
        --host: IP of the host server (default: localhost)
        -p, --port: TCP port to listen to (default: 2000)
        -a, --autopilot: enable autopilot
        -q, --quality-level: graphics quality level [Low or Epic]
        -i, --images-to-disk: save images to disk
        -c, --carla-settings: Path to CarlaSettings.ini file
    """
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Low',
        help='graphics quality level.')
    argparser.add_argument(
        '-c', '--carla-settings',
        metavar='PATH',
        dest='settings_filepath',
        default=None,
        help='Path to a "CarlaSettings.ini" file')
    argparser.add_argument(
        '--control-method',
        metavar='CONTROL_METHOD',
        dest='control_method',
        choices = {'PurePursuit','Stanley','MPC'},
        default='MPC',
        help='Select control method for Lane Keeping Assist : PurePursuit, Stanley or MPC')
    args = argparser.parse_args()

    file_path = 'C:\Shengjie\carla_indy\Wolverine_CARLA_Thunderhill\T01_waypoint.txt'
    wp_np,wp_list = importCSV(file_path)
    # print(wp_np)
    # print(wp_np.shape)
    wp_dist,wp_np_inter,hash_table = interpWaypoints(wp_np,0.01)
    # print(wp_dist)
    # print(len(wp_np_inter),len(hash_table))

    #############################################
    # Controller 2D Class Declaration
    #############################################
    # This is where we take the controller2d.py class
    # and apply it to the simulator
    controller = controller2d.Controller2D(wp_list, args.control_method)
    print("Wheel Base: ", controller._wheelbase)
    print("Speed: ", controller._current_speed)

    return


############################################################################
#                       Data Manipulation                                  #
#                                                                          #
############################################################################
def importCSV(WAYPOINTS_FILENAME):
    waypoints_file = WAYPOINTS_FILENAME
    waypoints_np   = None
    with open(waypoints_file) as waypoints_file_handle:
        waypoints = list(csv.reader(waypoints_file_handle,
                                    delimiter=',',
                                    quoting=csv.QUOTE_NONNUMERIC))
        waypoints_np = np.array(waypoints)
    return waypoints_np,waypoints

def interpWaypoints(waypoints_np,INTERP_DISTANCE_RES):
    # Because the waypoints are discrete and our controller performs better
    # with a continuous path, here we will send a subset of the waypoints
    # within some lookahead distance from the closest point to the vehicle.
    # Interpolating between each waypoint will provide a finer resolution
    # path and make it more "continuous". A simple linear interpolation
    # is used as a preliminary method to address this issue, though it is
    # better addressed with better interpolation methods (spline
    # interpolation, for example).

    # More appropriate interpolation methods will not be used here for the
    # sake of demonstration on what effects discrete paths can have on
    # the controller. It is made much more obvious with linear
    # interpolation, because in a way part of the path will be continuous
    # while the discontinuous parts (which happens at the waypoints) will
    # show just what sort of effects these points have on the controller.

    # Inputs: waypoints_numpy_datasets, interpolation_resolution(0.01 meter recommended,
    # once the input waypoints has a 1 meter interval.)
    # Outputs: interpolated waypoints, hash_table of 

    # Linear interpolation computations
    # Compute a list of distances between waypoints
    wp_distance = []   # distance array
    for i in range(1, waypoints_np.shape[0]):
        wp_distance.append(
                np.sqrt((waypoints_np[i, 0] - waypoints_np[i-1, 0])**2 +
                        (waypoints_np[i, 1] - waypoints_np[i-1, 1])**2))
    wp_distance.append(0)  # last distance is 0 because it is the distance
                            # from the last waypoint to the last waypoint

    # Linearly interpolate between waypoints and store in a list
    wp_interp      = []    # interpolated values
                            # (rows = waypoints, columns = [x, y, v])
    wp_interp_hash = []    # hash table which indexes waypoints_np
                            # to the index of the waypoint in wp_interp
    interp_counter = 0     # counter for current interpolated point index
    for i in range(waypoints_np.shape[0] - 1):
        # Add original waypoint to interpolated waypoints list (and append
        # it to the hash table)
        wp_interp.append(list(waypoints_np[i]))
        wp_interp_hash.append(interp_counter)
        interp_counter+=1

        # Interpolate to the next waypoint. First compute the number of
        # points to interpolate based on the desired resolution and
        # incrementally add interpolated points until the next waypoint
        # is about to be reached.
        num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                        float(INTERP_DISTANCE_RES)) - 1)
        wp_vector = waypoints_np[i+1] - waypoints_np[i]
        wp_uvector = wp_vector / np.linalg.norm(wp_vector)
        for j in range(num_pts_to_interp):
            next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
            wp_interp.append(list(waypoints_np[i] + next_wp_vector))
            interp_counter+=1
    # add last waypoint at the end
    wp_interp.append(list(waypoints_np[-1]))
    wp_interp_hash.append(interp_counter)
    interp_counter+=1
    return wp_distance,wp_interp,wp_interp_hash
############################################################################
#                       Waypoints Generation                               #
#                                                                          #
############################################################################


def _get_waypoints(trajectory):
    waypoints = []
    for index in range(len(trajectory)):
        waypoint = trajectory[index][0]
        waypoints.append([waypoint.location.x, waypoint.location.y, 0])
    return waypoints

def text_save(filename, data):#filename为写入CSV文件的路径，data为要写入数据列表.
    file = open(filename,'w')
    for i in range(len(data)):
        s = str(data[i]).replace('[','').replace(']','')#去除[],这两行按数据不同，可以选择
        s = s.replace("'",'').replace(',','') +'\n'   #去除单引号，逗号，每行末尾追加换行符
        file.write(s)
    file.close()
    print("保存文件成功") 

def draw_trajectory(trajectory, persistency=120, vertical_shift=0):
    for index in range(len(trajectory)):
        color_start = carla.Color(r=0, g=255, b=0, a =20)
        color = carla.Color(r=0, g=0, b=255, a =20)
        color_end = carla.Color(r=255, g=0, b=0, a =20)
        if index == 0:
            waypoint = trajectory[index][0]
            location = waypoint.location + carla.Location(z=vertical_shift)
            world.debug.draw_string(location, 'START', draw_shadow=False,
                                        color=color_start, life_time=persistency,
                                        persistent_lines=True)
        elif index == len(trajectory)-1:
            waypoint = trajectory[index][0]
            location = waypoint.location + carla.Location(z=vertical_shift)
            world.debug.draw_string(location, 'END', draw_shadow=False,
                                        color=color_end, life_time=persistency,
                                        persistent_lines=True)				
        else:
            waypoint = trajectory[index][0]
            location = waypoint.location + carla.Location(z=vertical_shift)
            world.debug.draw_string(location, '.', draw_shadow=False,
                                        color=color, life_time=persistency,
                                        persistent_lines=True)

def packWaypoints(waypoint_txt,world_input):
    world = world_input
    output_waypoint_list = []
    print(len(waypoint_txt))
    for index in range(len(waypoint_txt)):
        x = waypoint_txt[index][0] 
        y = waypoint_txt[index][1] 
        z = waypoint_txt[index][2] 
        location = carla.Vector3D(x, y, z)
        output_wp= world.get_map().get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Driving)
        # output_wp = carla.Waypoint(carla.Transform(carla.Location(x, y, z)))
        # print(x,y,z)
        # print(output_wp)
        # output_wp = sample_wp
        output_waypoint_list.append(output_wp)
    return output_waypoint_list

def getNexWaypoint(world, vehicle, waypoints):
    vehicle_location = vehicle.get_location()
    min_distance = 10
    waypoints_order = []
    next_waypoint = None
    for waypoint in waypoints[:section]:
        waypoint_location = waypoint.transform.location
        if vehicle_location.distance(waypoint_location) < min_distance:
            min_distance = vehicle_location.distance(waypoint_location)
            waypoints_order.append(waypoint)

    next_waypoint = waypoints_order[0]

    return next_waypoint

############################################################################
#                       Controller Group                                   #
#                                                                          #
############################################################################

def send_control_command(client, throttle, steer, brake,
                         hand_brake=False, reverse=False):
    """Send control command to CARLA client.
    NOTE: CARLA 0.8 only
    Send control command to CARLA client.

    Args:
        client: The CARLA client object
        throttle: Throttle command for the sim car [0, 1]
        steer: Steer command for the sim car [-1, 1]
        brake: Brake command for the sim car [0, 1]
        hand_brake: Whether the hand brake is engaged
        reverse: Whether the sim car is in the reverse gear
    """
    control = VehicleControl()
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    client.send_control(control)

def shiftCoordinate(args):
    # Gather current data from the CARLA server
    measurement_data, sensor_data = client.read_data()

    # Update pose, timestamp
    current_x, current_y, current_yaw = \
        get_current_pose(measurement_data)
    current_speed = measurement_data.player_measurements.forward_speed
    current_timestamp = float(measurement_data.game_timestamp) / 1000.0

    # Shift x, y coordinates
    if args.control_method == 'PurePursuit':
        length = -1.5
    elif args.control_method == 'Stanley' or args.control_method == 'MPC':
        length = 1.5
    else:
        length = 0.0
    current_x, current_y = controller.get_shifted_coordinate(current_x, current_y, current_yaw, length)
    return current_x, current_y

def updateCommand():
    # for the next controller update.
    new_waypoints = \
            wp_interp[wp_interp_hash[waypoint_subset_first_index]:\
                        wp_interp_hash[waypoint_subset_last_index] + 1]
    controller.update_waypoints(new_waypoints)

    # Update the other controller values and controls
    controller.update_values(current_x, current_y, current_yaw,
                                current_speed,
                                current_timestamp, frame, new_distance)
    controller.update_controls()
    cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
    return cmd_throttle, cmd_steer, cmd_brake

def control_pure_pursuit(vehicle_tr, waypoint_tr, max_steer, wheelbase):
    # TODO: convert vehicle transform to rear axle transform
    wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
    wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
    d2 = wp_ar[0]**2 + wp_ar[1]**2
    steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
    steer_deg = math.degrees(steer_rad)
    steer_deg = np.clip(steer_deg, -max_steer, max_steer)
    return steer_deg / max_steer

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

def single_lane(waypoint_list, lane):
    # Extract the choosen lane from the waypoints bundle.
    waypoints = []
    for i in range(len(waypoint_list) - 1):
        if waypoint_list[i].lane_id == lane:
            waypoints.append(waypoint_list[i])
    return waypoints
    
#Returns only the waypoints that are not along the straights
def get_curvy_waypoints(waypoints):
    curvy_waypoints = []
    for i in range(len(waypoints) - 1):
        x1 = waypoints[i].transform.location.x
        y1 = waypoints[i].transform.location.y
        x2 = waypoints[i+1].transform.location.x
        y2 = waypoints[i+1].transform.location.y
        if (abs(x1 - x2) > 1) and (abs(y1 - y2) > 1):
            print("x1: " + str(x1) + "  x2: " + str(x2))
            print(abs(x1 - x2))
            print("y1: " + str(y1) + "  y2: " + str(y2))
            print(abs(y1 - y2))
            curvy_waypoints.append(waypoints[i])
      
    #To make the path reconnect to the starting location
    curvy_waypoints.append(curvy_waypoints[0])

    return curvy_waypoints

def get_nearest_waypoints(vehicle,map):
    min_distance = 10000
    wp = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Shoulder))
    vehicle_location = vehicle.get_location()
    waypoint_location = wp.transform.location
    if (waypoint_location - vehicle_location).x > 0:
        min_distance = vehicle_location.distance(waypoint_location)
        if vehicle_location.distance(waypoint_location) < min_distance and vehicle_location.distance(waypoint_location) > 5:
            return wp

def control_pure_pursuit(vehicle_tr, waypoint_tr, max_steer, wheelbase):
    # TODO: convert vehicle transform to rear axle transform
    wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
    wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
    d2 = wp_ar[0]**2 + wp_ar[1]**2
    steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
    steer_deg = math.degrees(steer_rad)
    steer_deg = np.clip(steer_deg, -max_steer, max_steer)
    return steer_deg / max_steer

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

def get_next_waypoint(world, vehicle, waypoints):
    vehicle_location = vehicle.get_location()
    min_distance = 1000
    # search_area = waypoint_grid+0.5
    next_waypoint = None
    for waypoint in waypoints:
        waypoint_location = waypoint.transform.location

        #Only check waypoints that are in the front of the vehicle (if x is negative, then the waypoint is to the rear)
        #TODO: Check if this applies for all maps
        if (waypoint_location - vehicle_location).x > 0:
            #Find the waypoint closest to the vehicle, but once vehicle is close to upcoming waypoint, search for next one
            if vehicle_location.distance(waypoint_location) < min_distance and vehicle_location.distance(waypoint_location) > 5:
                min_distance = vehicle_location.distance(waypoint_location)

                if waypoint is not None:
                    # print("succeed")
                    next_waypoint = waypoint
                    pre_waypoint = next_waypoint
                else:
                    print("none")
                    next_waypoint = pre_waypoint
        # elif(waypoint_location - vehicle_location).x < 0:
        #     next_waypoints = waypoint.next(2)
        #     next_waypoint = next_waypoints[0]
        #     print(next_waypoint)
    # next_waypoint = next_waypoint.next(waypoint_search_grid)[0]
    # print(next_waypoint.transform.location.x,next_waypoint.transform.location.y)
    return next_waypoint

if __name__ == "__main__":

    try:
        # main()
        test_only()
    except KeyboardInterrupt:
        pass
    finally:
        print('destroying actors')
        # for actor in actor_list:
        #    actor.destroy()
        print('\ndone.')