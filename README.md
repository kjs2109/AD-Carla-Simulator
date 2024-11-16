# Introduction 
### Simulaion Loop 
### ACC System 
### Experiment Result 

# Prerequisite 
* OS: Ubuntu (18.04+)
* [pyenv](https://github.com/pyenv/pyenv) 

# Installation 
1. Clone Source Code
```
git clone https://github.com/kjs2109/AD-Carla-Simulator.git
```
3. Create Python Environment 
```
$ pyenv install 3.7 
$ pyenv local 3.7
$ pyenv rehash 

$ python -m venv carla-0913-env 
$ source carla-0913-env/bin/activate
```
3. Install Dependencies
```
pip install -r requirements.txt
```
4. Downlaod Custom Asset 
```
# 1. Carla package with the test road
$ cd ./carla_simulator 
$ tar -xf carla_simulator.tar.xz --strip-components=1

# 2. Object Detection Resources
$ cd ./resources 
$ tar -xf resources.tar.xz --strip-components=1
```
# Quick Start 
```
$ source set_carla_env.sh 

# simulation loop 
$ python simulation_loop.py 

# single scenario test 
$ ./carla_simulator/CarlaUE4.sh
$ python scenario_runner.py --openscenario ./scenario/ACC_change_velo.xosc --reloadWorld 

# manual control simulation 
$ ./carla_simulator/CarlaUE4.sh
$ python manual_control.py 
$ python scenario_runner.py --openscenario ./scenario/ACC_change_velo_manual_control.xosc --reloadWorld 
```

# Usage 
* The driving data from the simulation is saved in the ```./saved folder.```
