# Application of Automated Guided Vehicles in a Hospital Environment

**Course:** RO47005 - Planning and Decision Making @ TU Delft\
**Group 28**\
Fabio Boekel (4855892)\
Paula Gasco Claramunt (5725453)\
Thomas Hettasch (5836905)\
Kian Heus (4876768)

# TO INCLUDE:
* Video of MuJoCo simulation avoiding moving bed and reaching the goal
* Video of MuJoCo simulation with a disturbance
* Video of tree growing

## Description

<p float="left">
<img src="graphics/growing_tree.gif" width="400" height="400" />
<img src="graphics/car_driving.gif" width="600" height="400" />
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

To install the program, first clone this repository:
```
git clone git@github.com:kianheus/PDM_Project_Group_28.git
```
Then install the dependencies as outlined above.

 Please also ensure that the relevant python executable runs on a dedicated GPU (if available). If run on the CPU, the simulation will not run realtime.

## Executing program
If using a conda environment, run ```conda activate group28rrt``` first.

To run the main program run the following:
```
cd code
python RRTMain.py
```

Warning: the program can be slow when running the simulation from the integrated graphics card. Be sure to run the program on the dedicated card.  

## Modified packages

The following package is used to get a MJCF file of a RC car: https://github.com/prl-mushr/mushr_mujoco_ros
This package is modified to get the MuSHR RC car in a custom made hospital environment. 

https://grabcad.com/library/hospital-service-room-bed-2-new-version-1 
