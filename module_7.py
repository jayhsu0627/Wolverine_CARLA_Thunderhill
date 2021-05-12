#!/usr/bin/env python3

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA waypoint follower assessment client script.

A controller assessment to follow a given trajectory, where the trajectory
can be defined using way-points.

STARTING in a moment...
"""
from __future__ import print_function
from __future__ import division

# System level imports
import sys
import os
import argparse
import logging
import time
import math
import numpy as np
import csv
import matplotlib.pyplot as plt
import controller2d
import configparser
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

# Script level imports
sys.path.append(os.path.abspath(sys.path[0] + '/..'))
import live_plotter as lv   # Custom live plotting library
import carla
from datetime import datetime
import srunner	
from srunner.challenge.utils.route_manipulation import interpolate_trajectory

# from carla            import sensor
# from carla.client     import make_carla_client, VehicleControl
# from carla.settings   import CarlaSettings
# from carla.tcp        import TCPConnectionError
# from carla.controller import utils


######

def _get_waypoints(trajectory):
    waypoints = []
    for index in range(len(trajectory)):
        waypoint = trajectory[index][0]
        waypoints.append([waypoint.location.x, waypoint.location.y, 0])
    return waypoints

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
######

"""
Configurable params
"""
ITER_FOR_SIM_TIMESTEP  = 10     # no. iterations to compute approx sim timestep
WAIT_TIME_BEFORE_START = 5.00   # game seconds (time before controller start)
TOTAL_RUN_TIME         = 200.00 # game seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER     = 300    # number of frames to buffer after total runtime
NUM_PEDESTRIANS        = 0      # total number of pedestrians to spawn
NUM_VEHICLES           = 0      # total number of vehicles to spawn
SEED_PEDESTRIANS       = 0      # seed for pedestrian spawn randomizer
SEED_VEHICLES          = 0      # seed for vehicle spawn randomizer

WEATHERID = {
    "DEFAULT": 0,
    "CLEARNOON": 1,
    "CLOUDYNOON": 2,
    "WETNOON": 3,
    "WETCLOUDYNOON": 4,
    "MIDRAINYNOON": 5,
    "HARDRAINNOON": 6,
    "SOFTRAINNOON": 7,
    "CLEARSUNSET": 8,
    "CLOUDYSUNSET": 9,
    "WETSUNSET": 10,
    "WETCLOUDYSUNSET": 11,
    "MIDRAINSUNSET": 12,
    "HARDRAINSUNSET": 13,
    "SOFTRAINSUNSET": 14,
}
SIMWEATHER = WEATHERID["CLEARNOON"]     # set simulation weather

PLAYER_START_INDEX = 1      # spawn index for player (keep to 1)
FIGSIZE_X_INCHES   = 8      # x figure size of feedback in inches
FIGSIZE_Y_INCHES   = 8      # y figure size of feedback in inches
PLOT_LEFT          = 0.1    # in fractions of figure width and height
PLOT_BOT           = 0.1
PLOT_WIDTH         = 0.8
PLOT_HEIGHT        = 0.8

# WAYPOINTS_FILENAME = 'T01_waypoint.txt'  # waypoint file to load

# WAYPOINTS_FILENAME = 'all_waypoints_3.txt'  # waypoint file to load
# WAYPOINTS_FILENAME = 'T01_waypoint_all_2.txt'  # waypoint file to load
WAYPOINTS_FILENAME = 'stanford_waypoints_full.txt'  # waypoint file to load

# WAYPOINTS_FILENAME = 'racetrack_waypoints.txt'  # waypoint file to load

DIST_THRESHOLD_TO_LAST_WAYPOINT = 10.0  # some distance from last position before
                                       # simulation ends

# Path interpolation parameters
INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # lookahead path
INTERP_LOOKAHEAD_DISTANCE = 20   # lookahead in meters
INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points

# controller output directory
CONTROLLER_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) +\
                           '/controller_output/'

def make_carla_settings(args):
    """Make a CarlaSettings object with the settings we need.
    """
    settings = CarlaSettings()

    # There is no need for non-agent info requests if there are no pedestrians
    # or vehicles.
    get_non_player_agents_info = False
    if (NUM_PEDESTRIANS > 0 or NUM_VEHICLES > 0):
        get_non_player_agents_info = True

    # Base level settings
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=get_non_player_agents_info,
        NumberOfVehicles=NUM_VEHICLES,
        NumberOfPedestrians=NUM_PEDESTRIANS,
        SeedVehicles=SEED_VEHICLES,
        SeedPedestrians=SEED_PEDESTRIANS,
        WeatherId=SIMWEATHER,
        QualityLevel=args.quality_level)
    return settings

class Timer(object):
    """ Timer Class

    The steps are used to calculate FPS, while the lap or seconds since lap is
    used to compute elapsed time.
    """
    def __init__(self, period):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()
        self._period_for_lap = period

    def tick(self):
        self.step += 1

    def has_exceeded_lap_period(self):
        if self.elapsed_seconds_since_lap() >= self._period_for_lap:
            return True
        else:
            return False

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) /\
                     self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time

def get_current_pose(measurement):
    """Obtains current x,y,yaw pose from the client measurements

    Obtains the current x,y, and yaw pose from the client measurements.

    Args:
        measurement: The CARLA client measurements (from read_data())

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
    """
    x   = measurement.player_measurements.transform.location.x
    y   = measurement.player_measurements.transform.location.y
    yaw = math.radians(measurement.player_measurements.transform.rotation.yaw)

    return (x, y, yaw)

def get_start_pos(scene):
    """Obtains player start x,y, yaw pose from the scene

    Obtains the player x,y, and yaw pose from the scene.

    Args:
        scene: The CARLA scene object

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
    """
    x = scene.player_start_spots[0].location.x
    y = scene.player_start_spots[0].location.y
    yaw = math.radians(scene.player_start_spots[0].rotation.yaw)
    print(x,y,yaw)
    return (x, y, yaw)

def send_control_command(vehicle,throttle, steer, brake,
                         hand_brake=False, reverse=False):
    """Send control command to CARLA client.

    Send control command to CARLA client.

    Args:
        client: The CARLA client object
        throttle: Throttle command for the sim car [0, 1]
        steer: Steer command for the sim car [-1, 1]
        brake: Brake command for the sim car [0, 1]
        hand_brake: Whether the hand brake is engaged
        reverse: Whether the sim car is in the reverse gear
    """    
    # Clamp all values within their limits
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)
    control_para = carla.VehicleControl(throttle, steer, brake)
    control_para.hand_brake = hand_brake
    control_para.reverse = reverse
    vehicle.apply_control(control_para)

def create_controller_output_dir(output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

def store_trajectory_plot(graph, fname):
    """ Store the resulting plot.
    """
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)

    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, fname)
    graph.savefig(file_name)

def write_trajectory_file(x_list, y_list, v_list, t_list):
    create_controller_output_dir(CONTROLLER_OUTPUT_FOLDER)
    file_name = os.path.join(CONTROLLER_OUTPUT_FOLDER, 'trajectory.txt')

    with open(file_name, 'w') as trajectory_file:
        for i in range(len(x_list)):
            trajectory_file.write('%3.3f, %3.3f, %2.3f, %6.3f\n' %\
                                  (x_list[i], y_list[i], v_list[i], t_list[i]))

def make_carla_client(host,port):
    client = carla.Client(host, port)
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

    # Retrieve a snapshot of the world at this point in time.
    world_snapshot = world.get_snapshot()

    # Wait for the next tick and retrieve the snapshot of the tick.
    world_snapshot = world.wait_for_tick()

    # Register a callback to get called every time we receive a new snapshot.
    world.on_tick(lambda world_snapshot: world_snapshot)

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
    print('make_carla_client----Carla client connected.')

    transform = carla.Transform(carla.Location(x=18.30674744, y=191.2394257, z=0), carla.Rotation(pitch=0,yaw=0,roll=0))
    start_waypoint = map.get_waypoint(transform.location)

    # #Set spawning location as initial waypoint
    waypoint = start_waypoint
    blueprint = world.get_blueprint_library().filter('vehicle.*tt*')[0]
    blueprint.set_attribute('color', '255,255,246')
    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = carla.Rotation(pitch=0,yaw=90,roll=0)

    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    print("make_carla_client----SPAWNED!")
    vehicle.set_simulate_physics(True)
    vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.All | carla.VehicleLightState.LowBeam | carla.VehicleLightState.HighBeam | carla.VehicleLightState.Interior))

    #Add spectator camera to get the view to move with the car 
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-20,0,0))
    camera_transform = carla.Transform(carla.Location(x=-10,z=7), carla.Rotation(-20,0,0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    spectator.set_transform(camera.get_transform())
    spectator.set_transform(camera.get_transform())    
    print("make_carla_client----Camera Done!")
    return client, world, spectator, map , vehicle, camera

def exec_waypoint_nav_demo(args):
    """ Executes waypoint navigation demo.
    """
    if args.control_method == 'PurePursuit':
        print("Control method selected : Pure Pursuit\n")
    elif args.control_method == 'Stanley':
        print("Control method selected : Stanley\n")
    elif args.control_method == 'MPC':
        print("Control method selected : MPC\n")
    else:
        print("Control method selected : Unknown\n")
    print('mark3')

    # with make_carla_client(args.host, args.port) as client:
    client, world, spectator, map, vehicle, camera = make_carla_client(args.host,args.port)

    # #Add spectator camera to get the view to move with the car 
    # camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # # camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-20,0,0))
    # camera_transform = carla.Transform(carla.Location(x=-10,z=7), carla.Rotation(-20,0,0))
    # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # spectator.set_transform(camera.get_transform())

    print(client)
    print('Carla client connected.')
    # settings = make_carla_settings(args)
    print('mark4')
    # Now we load these settings into the server. The server replies
    # with a scene description containing the available start spots for
    # the player. Here we can provide a CarlaSettings object or a
    # CarlaSettings.ini file as string.
    # scene = client.load_settings(settings)
    # print(scene)
    # Refer to the player start folder in the WorldOutliner to see the
    # player start information
    # player_start = PLAYER_START_INDEX

    # Notify the server that we want to start the episode at the
    # player_start index. This function blocks until the server is ready
    # to start the episode.
    # print('Starting new episode at %r...' % scene.map_name)
    # client.start_episode(player_start)
    print('mark5')

    #############################################
    # Load Configurations
    #############################################

    # Load configuration file (options.cfg) and then parses for the various
    # options. Here we have two main options:
    # live_plotting and live_plotting_period, which controls whether
    # live plotting is enabled or how often the live plotter updates
    # during the simulation run.
    config = configparser.ConfigParser()
    config.read(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'options.cfg'))
    demo_opt = config['Demo Parameters']

    # Get options
    enable_live_plot = demo_opt.get('live_plotting', 'true').capitalize()
    enable_live_plot = enable_live_plot == 'True'
    live_plot_period = float(demo_opt.get('live_plotting_period', 0))

    # Set options
    live_plot_timer = Timer(live_plot_period)

    #############################################
    # Load Waypoints
    #############################################
    # Opens the waypoint file and stores it to "waypoints"
    waypoints_file = WAYPOINTS_FILENAME
    waypoints_np   = None
    with open(waypoints_file) as waypoints_file_handle:
        waypoints = list(csv.reader(waypoints_file_handle,
                                    delimiter=',',
                                    quoting=csv.QUOTE_NONNUMERIC))
        waypoints_np = np.array(waypoints)
    print('Waypoints loaded')
    # print(waypoints_np)

    for w in waypoints_np:
        world.debug.draw_string(carla.Location(x=w[0],y = w[1]), 'O', draw_shadow=False,
                                    color=carla.Color(r=255, g=255, b=0, a =200), life_time=500.0,
                                    persistent_lines=True)

    # print(waypoints_np[0][0],waypoints_np[0][1])
    # # Refresh Vehicle start location
    # transform = carla.Transform(carla.Location(x=waypoints_np[0][0], y=waypoints_np[0][1], z=0), carla.Rotation(pitch=0,yaw=0,roll=0))
    # print("1")
    # start_waypoint = map.get_waypoint(transform.location)
    # print(start_waypoint.transform)
    # blueprint = world.get_blueprint_library().filter('vehicle.*tt*')[0]
    # blueprint.set_attribute('color', '255,255,246')
    # print("2")
    # # location = start_waypoint.transform.location
    # print('location')
    # # rotation = carla.Rotation(pitch=0,yaw=90,roll=0)
    # print('rotation')
    # print(world,blueprint,start_waypoint.transform)
    # vehicle = world.spawn_actor(blueprint, start_waypoint.transform)
    # # vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    # print('vehicle')
    # vehicle.set_simulate_physics(True)
    # vehicle.set_light_state(carla.VehicleLightState(carla.VehicleLightState.All | carla.VehicleLightState.LowBeam | carla.VehicleLightState.HighBeam | carla.VehicleLightState.Interior))
    # print("333")

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
    # Can you spot these during the simulation? If so, how can you further
    # reduce these effects?

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

    print("numpy original: ",waypoints_np.shape[0])

    #
    delete_items = []
    for i in range(waypoints_np.shape[0] - 1):
        
        wp_vector = waypoints_np[i+1] - waypoints_np[i]
        if wp_vector[0] == 0:
            delete_items.append(i+1)

    waypoints_np = np.delete(waypoints_np, delete_items, 0)
    print("numpy deleted: ", waypoints_np.shape[0])
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
    print('Interpolate Finish')
    # add last waypoint at the end
    wp_interp.append(list(waypoints_np[-1]))
    wp_interp_hash.append(interp_counter)
    interp_counter+=1

    #############################################
    # Controller 2D Class Declaration
    #############################################
    # This is where we take the controller2d.py class
    # and apply it to the simulator
    # controller = controller2d.Controller2D(waypoints, args.control_method)
    controller = controller2d.Controller2D(waypoints, args.control_method, world)
    print('Controller loaded')
    #############################################
    # Determine simulation average timestep (and total frames)
    #############################################
    # Ensure at least one frame is used to compute average timestep
    num_iterations = ITER_FOR_SIM_TIMESTEP
    if (ITER_FOR_SIM_TIMESTEP < 1):
        num_iterations = 1
    print("Closed")
    # Gather current data from the CARLA server. This is used to get the
    # simulator starting game time. Note that we also need to
    # send a command back to the CARLA server because synchronous mode
    # is enabled.
    # measurement_data, sensor_data = client.read_data()
    
    timestamp = world.wait_for_tick()
    # print(timestamp.elapsed_seconds)

    # timestamp.frame_count
    # timestamp.elapsed_seconds
    # timestamp.delta_seconds
    # timestamp.platform_timestamp

    sim_start_stamp = timestamp.elapsed_seconds
    # sim_start_stamp = measurement_data.game_timestamp / 1000.0
    # Send a control command to proceed to next iteration.
    # This mainly applies for simulations that are in synchronous mode.
    send_control_command(vehicle,throttle=0.0, steer=0, brake=1.0)

    # Computes the average timestep based on several initial iterations
    sim_duration = 0
    for i in range(num_iterations):
        # Gather current data
        # measurement_data, sensor_data = client.read_data()
        timestamp = world.wait_for_tick()
        now_stamp = timestamp.elapsed_seconds
        # Send a control command to proceed to next iteration
        send_control_command(vehicle,throttle=0.0, steer=0, brake=1.0)
        # Last stamp
        if i == num_iterations - 1:
            print(now_stamp,sim_start_stamp)
            sim_duration = now_stamp - sim_start_stamp
    print(sim_duration)
    # Outputs average simulation timestep and computes how many frames
    # will elapse before the simulation should end based on various
    # parameters that we set in the beginning.

    SIMULATION_TIME_STEP = sim_duration / float(num_iterations)
    print("SERVER SIMULATION STEP APPROXIMATION: " + \
            str(SIMULATION_TIME_STEP))
    TOTAL_EPISODE_FRAMES = int((TOTAL_RUN_TIME + WAIT_TIME_BEFORE_START) /\
                            SIMULATION_TIME_STEP) + TOTAL_FRAME_BUFFER
    print("total frame: ",TOTAL_EPISODE_FRAMES)
    #############################################
    # Frame-by-Frame Iteration and Initialization
    #############################################
    # Store pose history starting from the start position
    # measurement_data, sensor_data = client.read_data()
    # start_x, start_y, start_yaw = get_current_pose(measurement_data)
    start_x = round(vehicle.get_transform().location.x,2)
    start_y = round(vehicle.get_transform().location.y,2)
    start_yaw = round(vehicle.get_transform().rotation.yaw,2)

    send_control_command(vehicle,throttle=0.0, steer=0, brake=1.0)
    x_history     = [start_x]
    y_history     = [start_y]
    yaw_history   = [start_yaw]
    time_history  = [0]
    speed_history = [0]
    print(start_x,start_y,start_yaw)
    #############################################
    # Vehicle Trajectory Live Plotting Setup
    #############################################
    # Uses the live plotter to generate live feedback during the simulation
    # The two feedback includes the trajectory feedback and
    # the controller feedback (which includes the speed tracking).
    lp_traj = lv.LivePlotter(tk_title="Trajectory Trace")
    lp_1d = lv.LivePlotter(tk_title="Controls Feedback")

    ###
    # Add 2D position / trajectory plot
    ###
    trajectory_fig = lp_traj.plot_new_dynamic_2d_figure(
            title='Vehicle Trajectory',
            figsize=(FIGSIZE_X_INCHES, FIGSIZE_Y_INCHES),
            edgecolor="black",
            rect=[PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT])

    trajectory_fig.set_invert_x_axis() # Because UE4 uses left-handed
                                        # coordinate system the X
                                        # axis in the graph is flipped
    trajectory_fig.set_axis_equal()    # X-Y spacing should be equal in size

    # Add waypoint markers
    trajectory_fig.add_graph("waypoints", window_size=waypoints_np.shape[0],
                                x0=waypoints_np[:,0], y0=waypoints_np[:,1],
                                linestyle="-", marker="", color='g')
    # Add trajectory markers
    trajectory_fig.add_graph("trajectory", window_size=TOTAL_EPISODE_FRAMES,
                                x0=[start_x]*TOTAL_EPISODE_FRAMES,
                                y0=[start_y]*TOTAL_EPISODE_FRAMES,
                                color=[1, 0.5, 0])
    # Add lookahead path
    trajectory_fig.add_graph("lookahead_path",
                                window_size=INTERP_MAX_POINTS_PLOT,
                                x0=[start_x]*INTERP_MAX_POINTS_PLOT,
                                y0=[start_y]*INTERP_MAX_POINTS_PLOT,
                                color=[0, 0.7, 0.7],
                                linewidth=4)
    # Add starting position marker
    trajectory_fig.add_graph("start_pos", window_size=1,
                                x0=[start_x], y0=[start_y],
                                marker=11, color=[1, 0.5, 0],
                                markertext="Start", marker_text_offset=1)
    # Add end position marker
    trajectory_fig.add_graph("end_pos", window_size=1,
                                x0=[waypoints_np[-1, 0]],
                                y0=[waypoints_np[-1, 1]],
                                marker="D", color='r',
                                markertext="End", marker_text_offset=1)
    # Add car marker
    trajectory_fig.add_graph("car", window_size=1,
                                marker="s", color='b', markertext="Car",
                                marker_text_offset=1)

    ###
    # Add 1D speed profile updater
    ###
    forward_speed_fig =\
            lp_1d.plot_new_dynamic_figure(title="Forward Speed (km/h)")
    forward_speed_fig.add_graph("forward_speed",
                                label="forward_speed",
                                window_size=TOTAL_EPISODE_FRAMES)
    forward_speed_fig.add_graph("reference_signal",
                                label="reference_Signal",
                                window_size=TOTAL_EPISODE_FRAMES)

    # Add throttle signals graph
    throttle_fig = lp_1d.plot_new_dynamic_figure(title="Throttle (%)")
    throttle_fig.add_graph("throttle",
                            label="throttle",
                            window_size=TOTAL_EPISODE_FRAMES)
    # Add brake signals graph
    brake_fig = lp_1d.plot_new_dynamic_figure(title="Brake (%)")
    brake_fig.add_graph("brake",
                            label="brake",
                            window_size=TOTAL_EPISODE_FRAMES)

    # Add steering signals graph
    steer_fig = lp_1d.plot_new_dynamic_figure(title="Steer (Degree)")
    steer_fig.add_graph("steer",
                            label="steer",
                            window_size=TOTAL_EPISODE_FRAMES)

    # live plotter is disabled, hide windows
    if not enable_live_plot:
        lp_traj._root.withdraw()
        lp_1d._root.withdraw()

    # Iterate the frames until the end of the waypoints is reached or
    # the TOTAL_EPISODE_FRAMES is reached. The controller simulation then
    # ouptuts the results to the controller output directory.
    reached_the_end = False
    skip_first_frame = True
    closest_index    = 0  # Index of waypoint that is currently closest to
                            # the car (assumed to be the first index)
    closest_distance = 0  # Closest distance of closest waypoint to car

    # #Add spectator camera to get the view to move with the car 
    # camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # # camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-20,0,0))
    # camera_transform = carla.Transform(carla.Location(x=-10,z=7), carla.Rotation(-20,0,0))
    # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)


    for frame in range(TOTAL_EPISODE_FRAMES):
        # Gather current data from the CARLA server
        # measurement_data, sensor_data = client.read_data()

        current_x = vehicle.get_transform().location.x
        current_y = vehicle.get_transform().location.y
        current_yaw = math.radians(vehicle.get_transform().rotation.yaw)
        # compensate for CoG 
        current_x, current_y = controller.get_shifted_coordinate(current_x, current_y, current_yaw, math.sqrt(4**2+0.5**2))

        # current_x = round(vehicle.get_transform().location.x,2)
        # current_y = round(vehicle.get_transform().location.y,2)
        # current_yaw = round(vehicle.get_transform().rotation.yaw,2)

        current_speed = math.sqrt(vehicle.get_velocity().x**2+vehicle.get_velocity().y**2)
        timestamp = world.wait_for_tick()
        current_timestamp = timestamp.elapsed_seconds

        spectator = world.get_spectator()
        transform = vehicle.get_transform()
        # spectator.set_transform(carla.Transform(transform.location + carla.Location(x=0,y = -10,z=20),
        # carla.Rotation(-20,90,0)))
 
        spectator.set_transform(camera.get_transform())

        # spectator.set_transform(carla.Transform(transform.location + carla.Location(x=0,y = -10,z=15),
        # carla.Rotation(-20,90,0)))

        # CAMERA SETTING


        # Update pose, timestamp
        # current_x, current_y, current_yaw = \
        #     get_current_pose(measurement_data)
        # current_speed = measurement_data.player_measurements.forward_speed
        # current_timestamp = float(measurement_data.game_timestamp) / 1000.0

        # Shift x, y coordinates
        if args.control_method == 'PurePursuit':
            # length = -1*controller.b
            length = -1.2
        elif args.control_method == 'Stanley' or args.control_method == 'MPC':
            # length = controller.a
            length = 1.2
        else:
            length = 0.0
        shifted_x, shifted_y = controller.get_shifted_coordinate(current_x, current_y, current_yaw, length)
        world.debug.draw_string(carla.Location(x=shifted_x,y = shifted_y), 'X', draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=255, a =255), life_time=0.01,
                            persistent_lines=True)
        # Wait for some initial time before starting the demo
        if current_timestamp <= WAIT_TIME_BEFORE_START:
            send_control_command(vehicle,throttle=0.0, steer=0, brake=1.0)
            continue
        else:
            current_timestamp = current_timestamp - WAIT_TIME_BEFORE_START

        # Store history
        x_history.append(current_x)
        y_history.append(current_y)
        yaw_history.append(current_yaw)
        speed_history.append(current_speed)
        time_history.append(current_timestamp)
        
        ###
        # Controller update (this uses the controller2d.py implementation)
        ###

        # To reduce the amount of waypoints sent to the controller,
        # provide a subset of waypoints that are within some
        # lookahead distance from the closest point to the car. Provide
        # a set of waypoints behind the car as well.

        # Find closest waypoint index to car. First increment the index
        # from the previous index until the new distance calculations
        # are increasing. Apply the same rule decrementing the index.
        # The final index should be the closest point (it is assumed that
        # the car will always break out of instability points where there
        # are two indices with the same minimum distance, as in the
        # center of a circle)
        closest_distance = np.linalg.norm(np.array([
                waypoints_np[closest_index, 0] - current_x,
                waypoints_np[closest_index, 1] - current_y]))

        new_distance = closest_distance
        # print(new_distance)
        new_index = closest_index
        while new_distance <= closest_distance:
            closest_distance = new_distance
            closest_index = new_index
            new_index += 1
            if new_index >= waypoints_np.shape[0]:  # End of path
                break
            new_distance = np.linalg.norm(np.array([
                    waypoints_np[new_index, 0] - current_x,
                    waypoints_np[new_index, 1] - current_y]))
        new_distance = closest_distance
        new_index = closest_index
        while new_distance <= closest_distance:
            closest_distance = new_distance
            closest_index = new_index
            new_index -= 1
            if new_index < 0:  # Beginning of path
                break
            new_distance = np.linalg.norm(np.array([
                    waypoints_np[new_index, 0] - current_x,
                    waypoints_np[new_index, 1] - current_y]))
        # print('mark00')


        # Once the closest index is found, return the path that has 1
        # waypoint behind and X waypoints ahead, where X is the index
        # that has a lookahead distance specified by
        # INTERP_LOOKAHEAD_DISTANCE
        waypoint_subset_first_index = closest_index - 1
        if waypoint_subset_first_index < 0:
            waypoint_subset_first_index = 0

        waypoint_subset_last_index = closest_index
        total_distance_ahead = 0
        while total_distance_ahead < INTERP_LOOKAHEAD_DISTANCE:
            total_distance_ahead += wp_distance[waypoint_subset_last_index]
            waypoint_subset_last_index += 1
            if waypoint_subset_last_index >= waypoints_np.shape[0]:
                waypoint_subset_last_index = waypoints_np.shape[0] - 1
                break

        # print('mark')

        #print("length of wp_interp",len(wp_interp))
        #print("new_index :",new_index)
        #print("waypoint_subset_first_index :",waypoint_subset_first_index)
        #print("waypoint_subset_last_index :",waypoint_subset_last_index)
        #print("wp_interp_hash[waypoint_subset_first_index] :",wp_interp_hash[waypoint_subset_first_index])
        #print("wp_interp_hash[waypoint_subset_last_index] :",wp_interp_hash[waypoint_subset_last_index])


        # Use the first and last waypoint subset indices into the hash
        # table to obtain the first and last indicies for the interpolated
        # list. Update the interpolated waypoints to the controller
        # for the next controller update.
        new_waypoints = \
                wp_interp[wp_interp_hash[waypoint_subset_first_index]:\
                            wp_interp_hash[waypoint_subset_last_index] + 1]
        controller.update_waypoints(new_waypoints)

        # Update the other controller values and controls
        controller.update_values(current_x, current_y, current_yaw,
                                    current_speed,
                                    current_timestamp, frame, new_distance)
        # print("update____")
        controller.update_controls()
        cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()

        # Skip the first frame (so the controller has proper outputs)
        if skip_first_frame and frame == 0:
            pass
        else:
            # Update live plotter with new feedback
            trajectory_fig.roll("trajectory", current_x, current_y)
            trajectory_fig.roll("car", current_x, current_y)
            # When plotting lookahead path, only plot a number of points
            # (INTERP_MAX_POINTS_PLOT amount of points). This is meant
            # to decrease load when live plotting
            new_waypoints_np = np.array(new_waypoints)
            path_indices = np.floor(np.linspace(0,
                                                new_waypoints_np.shape[0]-1,
                                                INTERP_MAX_POINTS_PLOT))

            trajectory_fig.update("lookahead_path",
                    new_waypoints_np[path_indices.astype(int), 0],
                    new_waypoints_np[path_indices.astype(int), 1],
                    new_colour=[0, 0.7, 0.7])

            forward_speed_fig.roll("forward_speed",
                                    current_timestamp,
                                    current_speed*3.6) # m/s to km/h
            forward_speed_fig.roll("reference_signal",
                                    current_timestamp,
                                    controller._desired_speed*3.6) # m/s to km/h

            throttle_fig.roll("throttle", current_timestamp, cmd_throttle*100)
            brake_fig.roll("brake", current_timestamp, cmd_brake*100)
            steer_fig.roll("steer", current_timestamp, cmd_steer*180/np.pi)

            # Refresh the live plot based on the refresh rate
            # set by the options
            if enable_live_plot and \
                live_plot_timer.has_exceeded_lap_period():
                lp_traj.refresh()
                lp_1d.refresh()
                live_plot_timer.lap()

        # Output controller command to CARLA server
        send_control_command(vehicle,throttle=cmd_throttle,
                                steer=cmd_steer*0.5,
                                brake=cmd_brake)

        # Find if reached the end of waypoint. If the car is within
        # DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint,
        # the simulation will end.
        dist_to_last_waypoint = np.linalg.norm(np.array([
            waypoints[-1][0] - current_x,
            waypoints[-1][1] - current_y]))
        print("dist_to_last_waypoint:",dist_to_last_waypoint)
        if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
            reached_the_end = True
            print("End")
            break
        # if reached_the_end:
            # break

    # End of demo - Stop vehicle and Store outputs to the controller output
    # directory.
    if reached_the_end:
        print("Reached the end of path. Writing to controller_output...")
    else:
        print("Exceeded assessment time. Writing to controller_output...")
    # Stop the car
    send_control_command(vehicle,throttle=0.0, steer=0.0, brake=1.0)
    # Store the various outputs
    store_trajectory_plot(trajectory_fig.fig, 'trajectory.png')
    store_trajectory_plot(forward_speed_fig.fig, 'forward_speed.png')
    store_trajectory_plot(throttle_fig.fig, 'throttle_output.png')
    store_trajectory_plot(brake_fig.fig, 'brake_output.png')
    store_trajectory_plot(steer_fig.fig, 'steer_output.png')
    write_trajectory_file(x_history, y_history, speed_history, time_history)

def main():
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
        default='127.0.0.1',
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

    print(args)
    ##Modifiable Variables from -1 to -5, as left to right
    # targetLane = -3
    targetLane = -3

    # Logging startup info
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = '_out/episode_{:0>4d}/{:s}/{:0>6d}'

    # Execute when server connection is established
    while True:
        try:
            exec_waypoint_nav_demo(args)
            print('mark2')
            print(args)
            print('Done.')
            return

        except:
            print("break out")
            break
        # TCPConnectionError as error:
        #     logging.error(error)
        #     time.sleep(1)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')