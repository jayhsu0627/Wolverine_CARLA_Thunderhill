#!/usr/bin/python
# -*- coding: utf-8 -*

import glob
import os
import sys
from datetime import datetime
try:
    sys.path.append(glob.glob('C:\carla\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
# import scenario_runner
import srunner	
#Import the library Transform used to explicitly spawn an actor
from carla import Transform, Location, Rotation
from carla import Map
from carla import Vector3D

from srunner.challenge.utils.route_manipulation import interpolate_trajectory

import random
import time

actor_list = []

try:


	##General connection to server and get blue_print
	client = carla.Client('localhost',2000)
	client.set_timeout(20)

	# world = client.get_world()
	# world = client.load_world('Town04') #change maps
    # xodr_path = "speedway_5lanes.xodr"
	xodr_path = "Thunderhilll.xodr"
	od_file = open(xodr_path)
	data = od_file.read()

	# # Load the opendrive map
	vertex_distance = 2.0  # in meters
	max_road_length = 50.0 # in meters  10
	wall_height = 0.0     # in meters  1.0
	extra_width = 0.6      # in meters
	global waypoint_grid
	waypoint_grid = 5
	# global waypoint_search_grid
	# waypoint_search_grid = 10
	throttle_value = 0.7
	initialized_point = 0
	constant_speed = 10 # m/s
	now = datetime.now()
	current_hours = int(now.strftime('%H'))/24*180


	world = client.generate_opendrive_world(
		data, carla.OpendriveGenerationParameters(
		vertex_distance=vertex_distance,
		max_road_length=max_road_length,
		wall_height=wall_height,
		additional_width=extra_width,
		smooth_junctions=True,
		enable_mesh_visibility=True))
	# world = client.load_world('Town02')

	weather = carla.WeatherParameters(
	cloudiness=0.0,
	precipitation=0.0,
	sun_altitude_angle=current_hours)
	world.set_weather(weather)
	spectator = world.get_spectator()

	map = world.get_map()


	spectator = world.get_spectator()

	mp = world.get_map()#get the map of the current world.
	blueprint_library = world.get_blueprint_library()
	
	transform = carla.Transform(carla.Location(x=13, y=-0.01, z=0), carla.Rotation(pitch=0,yaw=180,roll=0))
	map = world.get_map()
	end_waypoint= map.get_waypoint(transform.location)
	# end_waypoint = start_waypoint.next(4556)[-1]
	start_waypoint= map.get_waypoint(transform.location + carla.Vector3D(0, -30, 0))
	
	waypoints = [start_waypoint.transform.location, end_waypoint.transform.location]
	gps_route, trajectory = interpolate_trajectory(world,waypoints,0.8)
	# print(gps_route)
	print(len(gps_route))
	print("**************************************")
	# print(trajectory)
	print(len(trajectory))

	#print(_get_waypoints(trajectory))
	

	def _get_waypoints(trajectory):
		waypoints = []
		for index in range(len(trajectory)):
			waypoint = trajectory[index][0]
			# print(waypoint)
			# waypoints.append([waypoint['lat'], waypoint['lon'], waypoint['z']])
			waypoints.append([waypoint.location.x, waypoint.location.y, 20])
		return waypoints
	
	
	def text_save(filename, data):#filename为写入CSV文件的路径，data为要写入数据列表.
		file = open(filename,'w')
		for i in range(len(data)):
			s = str(data[i]).replace('[','').replace(']','')#去除[],这两行按数据不同，可以选择
			# s = s.replace("'",'').replace(',','') +'\n'
			s = s.replace("'",'').replace(" ",'') +'\n'   #去除单引号，逗号，每行末尾追加换行符
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
	waypoints_list = _get_waypoints(trajectory)
	filename = 'C:\Shengjie\carla_indy\T02_waypoint.txt'
	text_save(filename, waypoints_list)
	draw_trajectory(trajectory,200,0.5)

finally:
	for actor in actor_list:
		actor.destroy()
	print("All cleaned up!")