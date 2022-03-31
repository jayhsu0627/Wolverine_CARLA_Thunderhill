# Wolverine_CARLA_Thunderhill

## What's this project about?
### Intution
This project is inspired by[ Nitin Kapania's Stanford PhD thesis](https://github.com/nkapania/Wolverine), [Final project for course 1 of the Coursera: Introduction to Self-Driving Cars](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081), and [Charlotte Dorn's Indianapolis waypoint tracking project ](https://medium.com/@chardorn/creating-carla-waypoints-9d2cc5c6a656). What if we run a vitual Audi TT in Thunderhill racetrack to test Kapania's path generation algorithm? 

### Where hosted

#### About Thunderhill

*"Thunderhill Raceway Park is a motorsports complex located 7 miles West of Willows, California, United States, in the Sacramento Valley. It is the venue for the longest automobile race in the United States, the 25 Hours of Thunderhill, held annually during the first weekend in December.*

*Thunderhill has two tracks: the original 3 mile track known as Thunderhill East and a 2 mile track known as Thunderhill West. The two tracks can also be combined to offer a 5-mile track, the longest road course in America. Thunderhill also offers two large skid pad areas as well for drifting and car control events. ---- Source: [Wikipedia](https://en.wikipedia.org/wiki/Thunderhill_Raceway_Park)"*

#### Virtual Thunderhill Playground

**Our Thunderhill but in CARLA 0.9.10**

| Real Thunderhill                                             | Ours                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/thil-from-airl.jpg) | ![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/birdsview.png) |

### Stanford Audi TTS

*"Shelley, as the self-driving car is known, is the product of collaboration between Stanford's Dynamic Design Lab, led by mechanical engineering Associate Professor Chris Gerdes, and the Volkswagen Electronics Research Lab. Earlier this summer, Gerdes' group brought Shelley to Thunderhill for high-speed tests of the latest tweaks to the software that tells her when to brake, how tight to take turns and when to punch the gas. ---- Source: Stanford Report"*


| **Audi TT**           |                   |Audi TTS 'Shelley'   |
| --------------------- | ----------------- |-------------------- |
|2.0 L I-4              |**Engine**         | 2.0 L I-4           |
|23/30 mpg (city/hwy)   |**Fuel Efficiency**|23/27 mpg (city/hwy) |
|220 @ 4500 rpm	        |**Horsepower**     |292 @ 5400 rpm       |
|258 ft-lbs @ 1600 rpm	|**Torque**         | 280 ft-lbs @ 1900 rpm|
|6-speed auto	        |**Transmission**   |6-speed auto	      |
|5.3 sec	            |**0-to-60 time**   |4.6 sec	          |
| ![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/Audi_TT.png) |**Apperance**   |![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/stanford-audi-tts.jpg)	      |

#### Video Archive:

<p align="center">
    <a href="http://www.youtube.com/watch?v=YxHcJTs2Sxk">
    <img width="460" height="300" 
    src="http://img.youtube.com/vi/YxHcJTs2Sxk/0.jpg"
    alt="Shelley, Stanford's Robotic Car, Hits the Track">
</p>

    Shelley, Stanford's Robotic Car, Hits the Track

<p align="center">
    <a href="https://www.youtube.com/watch?v=WWSorkMbsC8">
    <img width="460" height="300" 
    src="http://img.youtube.com/vi/WWSorkMbsC8/0.jpg"
    alt="Pure Pursuit">
</p>

    Our replication is 50% of the max speed running by the pure pursuit lateral control method.

<p align="center">
    <a href="https://www.youtube.com/watch?v=kCZA5xsIFg8">
    <img width="460" height="300" 
    src="http://img.youtube.com/vi/kCZA5xsIFg8/0.jpg"
    alt="Shelley, Stanford's Robotic Car, Hits the Track">
</p>

    This replication is running by the Stanley lateral control method with max speed of 150 kph.



****
## Backbone
****
## Get Started

### Requirements

Copied from [CARLA Documentation](https://carla.readthedocs.io/en/0.9.11/start_quickstart/#requirements).

- **Server side.** A **4GB minimum GPU** will be needed to run a highly realistic environment. A dedicated GPU is highly advised for machine learning. **(Note: ours has three GTX 1080Ti, with 12GB for each GPU)**
- **Client side.** [Python](https://www.python.org/downloads/) is necessary to access the API via command line. Also, a good internet connection and two TCP ports (2000 and 2001 by default).
- **System requirements.** Any 64-bits OS should run CARLA. However, since release 0.9.9, **CARLA cannot run in 16.04 Linux systems with default compilers**. These should be upgraded to work with CARLA.
- **Other requirements.** Two Python modules: ~~[Pygame](https://pypi.org/project/pygame/) to create graphics directly with Python~~, and [Numpy](https://pypi.org/project/numpy/) for great calculus.

### Environment

1. Download **CARLA 0.9.10** compiled package from [here](https://github.com/carla-simulator/carla/releases/tag/0.9.10), choose CARLA_0.9.10.zip.
2. Build your anaconda environment, dependency see envname.yml. Our environment is built with miniconda and a python=3.7 environment.

```
# Download your CARLA_0.9.10.zip
# Extract it
# Navigate to your carla folder
cd \your\path\to\carla\CARLA_0.9.10\WindowsNoEditor
CarlaUE4.exe

# Create an environment named py37 from YAML file
conda env create --file envname.yml

# Activate your environment
conda activate py37

```

you'll get

![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/UE4_default_page.png)

Remember, to ``` import carla``` , you always need to paste the the following before your any client scripts. The ```*.egg``` include all necessary package to convert your python3.7 script into CARLA c++ environment. More about this issue, see [this](https://carla.readthedocs.io/en/latest/build_system/).

```
try:
    sys.path.append(glob.glob('/your/path/to/carla/CARLA_0.9.10/WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
```

SRunner

edit PATH environment



### Map Generation

#### OpenStreetMap

Click the following picture, jump to OpenStreetMap in your browser.

<p align="center">
    <a href="https://www.openstreetmap.org/#map=16/39.5387/-122.3368">
    <img width="" height="300" 
    src="https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/map.png"
    alt="Thunderhill in OSM">
</p>


Copy following code to view in HTML



```HTML
<iframe width="425" height="350" frameborder="0" scrolling="no" marginheight="0" marginwidth="0" src="https://www.openstreetmap.org/export/embed.html?bbox=-122.35038042068483%2C39.530773993553694%2C-122.32338666915895%2C39.546560835379374&amp;layer=mapnik" style="border: 1px solid black"></iframe><br/><small><a href="https://www.openstreetmap.org/#map=16/39.5387/-122.3369">Check Larger Map</a></small>
```


#### RoadRunner![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/roadrunner.png)

*"RoadRunner is an interactive editor that lets you design 3D scenes for simulating and testing automated driving systems. You can customize roadway scenes by creating region-specific road signs and markings. You can insert signs, signals, guardrails, and road damage, as well as foliage, buildings, and other 3D models. RoadRunner provides tools for setting and configuring traffic signal timing, phases, and vehicle paths at intersections.----MathWorks"*


### Scripts
#### Wolverine (path generation)
You can try my Colab notebook by click

[!["Stanford.ipynb" Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/11XgI3dEN68-wI52tvl7awQ_fPqkG-qge?usp=sharing)

Generated path.
![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/generated_path.png)

While our simulation video uses the centerline of the track as the tracking target for the control algorithm, what Colab generates is the optimal path that simulates a real racer. In the follow-up work, it can be considered to import the optimized path into the simulation environment for testing, but the apex of the road model needs to be optimized.

#### Navigate and Control

```
# Use PurePursuit to control
python Try_w.py --control-method PurePursuit

# Use Stanley to control
python Try_w.py --control-method Stanley

# Use MPC to control
python Try_w.py --control-method MPC
```

<!-- #### Easter egg
##### Dynamic change sun angle -->



****
Reference:

1. [Path following and path generation framework from Nitin Kapania's Stanford PhD packed in python](https://github.com/nkapania/Wolverine)
2. [Creating Carla Waypoints in an Indianapolis racetrack ](https://medium.com/@chardorn/creating-carla-waypoints-9d2cc5c6a656)
3. [Shelley, Stanford's robotic racecar, hits the track ](https://news.stanford.edu/news/2012/august/shelley-autonomous-car-081312.html)
4. [Our Very Own Grand Challenge](https://medium.com/udacity/our-very-own-grand-challenge-b004a9863024)
5. [Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081)
6.  [ScenarioRunner Getting started](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_scenariorunner.md)
7.  [Homepage of RoadRunner](https://www.mathworks.com/products/roadrunner.html)