# Application of Automated Guided Vehicles in a Hospital Environment

**Course:** RO47005 - Planning and Decision Making\
**Group 28**\
Fabio Boekel (4855892)\
Paula Gasco Claramunt (5725453)\
Thomas Hettasch (5836905)\
Kian Heus (4876768)\

## Description


## Dependencies

* Prerequisites, libraries, etc

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

## Installation

* How/where to download your program
* Any modifications needed to be made to files/folders

Process:
```
git clone https://github.com/kianheus/PDM_Legends

conda env create -f environment.yml
```
 Please also ensure that the newly created python executable (for the conda environment) executes on a dedicated GPU (if available). If run on the CPU, the simulation is very slow.

## Executing program

* How to run the program
* Step-by-step instructions
```
cd code
conda activate group28rrt
python RRTMain.py
```