# Wolverine_CARLA_Thunderhill

## What's this project about?
### Intution
This project is inspired by[ Nitin Kapania's Stanford PhD thesis](https://github.com/nkapania/Wolverine), and [Charlotte Dorn's Indianapolis waypoint tracking project ](https://medium.com/@chardorn/creating-carla-waypoints-9d2cc5c6a656).

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


| **Audi TT**           |       |Audi TTS 'Shelley'   |
| --------------------- | ----  |-------------------- |
|2.0 L I-4              |**Engine**         | 2.0 L I-4           |
|23/30 mpg (city/hwy)   |**Fuel Efficiency**|23/27 mpg (city/hwy) |
|220 @ 4500 rpm	        |**Horsepower**     |292 @ 5400 rpm       |
|258 ft-lbs @ 1600 rpm	|**Torque**         | 280 ft-lbs @ 1900 rpm|
|6-speed auto	        |**Transmission**   |6-speed auto	      |
|5.3 sec	        |**0-to-60 time**   |4.6 sec	      |
|5.3 sec	        |**Apperance**   |![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/stanford-audi-tts.jpg)	      |

#### Video Archive:

<p align="center">
    <a href="http://www.youtube.com/watch?v=YxHcJTs2Sxk">
    <img width="460" height="300" 
    src="http://img.youtube.com/vi/YxHcJTs2Sxk/0.jpg"
    alt="Shelley, Stanford's Robotic Car, Hits the Track">
</p>



****
## Backbone
****
## Get Started

### Environment
1. Download CARLA 0.9.10 compiled package from [here](https://github.com/carla-simulator/carla/releases/tag/0.9.10), choose CARLA_0.9.10.zip.
2. Build your anaconda environment, dependency see envname.yml

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

### Map Generation

#### OpenStreetMap

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


#### RoadRunner

*"RoadRunner is an interactive editor that lets you design 3D scenes for simulating and testing automated driving systems. You can customize roadway scenes by creating region-specific road signs and markings. You can insert signs, signals, guardrails, and road damage, as well as foliage, buildings, and other 3D models. RoadRunner provides tools for setting and configuring traffic signal timing, phases, and vehicle paths at intersections.----MathWorks"*


### Scripts
#### Wolverine (path generation)
You can try my Colab notebook by click

[!["Stanford.ipynb" Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/11XgI3dEN68-wI52tvl7awQ_fPqkG-qge?usp=sharing)


#### Navigate and Control


****
see

1. [Path following and path generation framework from Nitin Kapania's Stanford PhD packed in python](https://github.com/nkapania/Wolverine)
2. [Creating Carla Waypoints in an Indianapolis racetrack ](https://medium.com/@chardorn/creating-carla-waypoints-9d2cc5c6a656)
3. [Shelley, Stanford's robotic racecar, hits the track ](https://news.stanford.edu/news/2012/august/shelley-autonomous-car-081312.html)
4. [Our Very Own Grand Challenge](https://medium.com/udacity/our-very-own-grand-challenge-b004a9863024)
5. [ScenarioRunner Getting started](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_scenariorunner.md)
6. [Homepage of RoadRunner](https://www.mathworks.com/products/roadrunner.html)


