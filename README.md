# Wolverine_CARLA_Thunderhill

## What's this project about?
### Intution
This project is inspired by:
1.[Path following and path generation framework from Nitin Kapania's Stanford PhD packed in python](https://github.com/nkapania/Wolverine)
2.[Creating Carla Waypoints in an Indianapolis racetrack ](https://medium.com/@chardorn/creating-carla-waypoints-9d2cc5c6a656)

### Where hosted
**Thunderhill but in CARLA 0.9.10**

**Real**

<p align="center">
    <a href="">
    <img width="" height="500" 
    src="https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/thil-from-airl.jpg"
    alt="thil-from-airl">
</p>


**Ours**

<p align="center">
    <a href="">
    <img width="" height="500"
	src="https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/birdsview.png"
    alt="birdsview">
</p>


### Stanford Audi TTS


<p align="center">
    <a href="">
    <img width="" height="500" 
    src="https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/stanford-audi-tts.jpg"
    alt="Stanford Audi TTS">
</p>

### Reference:

<p align="center">
    <a href="http://www.youtube.com/watch?v=YxHcJTs2Sxk">
    <img width="460" height="300" 
    src="http://img.youtube.com/vi/YxHcJTs2Sxk/0.jpg"
    alt="Shelley, Stanford's Robotic Car, Hits the Track">
</p>

[Our Very Own Grand Challenge](https://medium.com/udacity/our-very-own-grand-challenge-b004a9863024)


## Backbone

## Get Started

### Environment
1. Download CARLA 0.9.10 compiled package from [here](https://github.com/carla-simulator/carla/releases/tag/0.9.10), choose CARLA_0.9.10.zip.
2. Build your anaconda environment

```
# Download your CARLA_0.9.10.zip
# Extract it
# Navigate to your carla folder
cd \your\path\to\carla\CARLA_0.9.10\WindowsNoEditor
CarlaUE4.exe


# Create an environment from YAML file
conda env create --file envname.yml
```

you'll get

![](https://github.com/jayhsu0627/Wolverine_CARLA_Thunderhill/blob/main/pics/UE4_default_page.png)

### Map Generation

#### OpenStreeMap

<p>
<iframe width="425" height="350" frameborder="0" scrolling="no" marginheight="0" marginwidth="0" src="https://www.openstreetmap.org/export/embed.html?bbox=-122.35038042068483%2C39.530773993553694%2C-122.32338666915895%2C39.546560835379374&amp;layer=mapnik" style="border: 1px solid black"></iframe><br/><small><a href="https://www.openstreetmap.org/#map=16/39.5387/-122.3369">Check Larger Map</a></small>
</p>


#### RoadRunner

*RoadRunner is an interactive editor that lets you design 3D scenes for simulating and testing automated driving systems. You can customize roadway scenes by creating region-specific road signs and markings. You can insert signs, signals, guardrails, and road damage, as well as foliage, buildings, and other 3D models. RoadRunner provides tools for setting and configuring traffic signal timing, phases, and vehicle paths at intersections.----MathWorks*

see [Homepage of RoadRunner](https://www.mathworks.com/products/roadrunner.html)

see 


### Scripts
#### Wolverine (path generation)
You can try my Colab notebook by click

[!["Stanford.ipynb" Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/11XgI3dEN68-wI52tvl7awQ_fPqkG-qge?usp=sharing)


#### Navigate and Control



see

1. [ScenarioRunner Getting started](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_scenariorunner.md)

