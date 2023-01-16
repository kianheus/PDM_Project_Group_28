# Application of Automated Guided Vehicles in a Hospital Environment

**Course:** RO47005 - Planning and Decision Making @ TU Delft\
**Group 28**\
Fabio Boekel (4855892)\
Paula Gasco Claramunt (5725453)\
Thomas Hettasch (5836905)\
Kian Heus (4876768)

## Description
This repository contains the source files for the project of team 28.
* Robot morphology: Non holonomic four wheeled vehicle.
* Motion planner: Reverse RRT* as global planner. Pre-grown RRT as local planner.
* Environment: Hospital world with four rooms, static beds and a moving bed. Full physics based enivornment using MuJoCo physics.
* Video of vehicle driving: https://youtu.be/t0rsL7pqINs
* Video of vehicle being disturbed: https://youtu.be/Lc2vbbH4lp4

<p float="left">
<img src="graphics/growing_tree.gif" width="35%" />
<img src="graphics/car_driving.gif" width="60%" />
</p>
 
## Dependencies
This repository requires the following python modules to work correctly:
```
numpy
matplotlib
tqdm
mujoco
gym
mujoco-python-viewer
scipy
pillow
pyyaml
```

The dependencies can be installed manually via pip, or using ```
conda env create -f environment.yml```. If conda is used, run ```conda activate group28rrt``` before running the program.

## Installation

To install the program with the conda environment, first clone this repository in desired location with the git command or download the zip and put in the prefered folder:
```
git clone https://github.com/kianheus/PDM_Project_Group_28.git
```
Then install the dependencies:
```
cd PDM_Project_Group_28/
conda env create -f environment.yml
```

*Warning: Please ensure that the relevant python executable runs on a dedicated GPU (if available). If run on the CPU or with integrated GPU, the simulation will not run realtime. For windows users using NVIDIA GPU, this can be done with the NVIDIA control panel*  

## Executing program
#### Running the program
If using a conda environment, run ```conda activate group28rrt``` first.

To run the main program run the following lines from the ```PDM_Project_Group_28 folder```:
```
cd code
python RRTMain.py
```
#### Interacting with the program
Instruction to interact with environment:
* In first camera view, the mouse can be used to look around the environment.
* Double click car. Hold '**ctrl**' while pressing '**left -**' or '**right mouse button**' to disturb the car. See visualisation below.
* Press '**tab**' to change camera
* Press '**space**' to pause the simulation
* Press '**h**' to open help menu for more options

<p float="left">
<img src="graphics/car_disturbed.gif" width="60%" />
</p>

#### Changing the program
In ```RRTMain.py``` some parameters including the start and end position can be changed. To change start and goal location, go to the '**class consts**' section and change '**start_pose**' and/or '**end_pose**'. 

*Note 1: if start position is changed, the tree does not have to be regrown. If the end position is changed, it will look into the files to check if that specific tree is already grown. If that tree is not grown, it will grow a tree of a limited size or when the time limit of 5 minutes is reached.*

*Note 2: if other parameters are changed that influence the tree, all the three always need to be (re)grown. In otherwords the pickle files in the folder trees need to be removed.*

## External packages/files used
* Gym: https://www.gymlibrary.dev/
* Physics engine: https://mujoco.org/
* MJCF file of a RC car: https://github.com/prl-mushr/mushr_mujoco_ros
    * This package is modified. Only MuSHR RC car is extracted from it and it is put in a custom made hospital environment. Also some small things are added to the car like sensors, colors and camera's.
* Interactive renderer that works and interacts with MuJoCo: https://github.com/rohanpsingh/mujoco-python-viewer
* STL file of hosptial bed: https://grabcad.com/library/hospital-service-room-bed-2-new-version-1 

