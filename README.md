# rocket-nav
Bridges RocketPy and gnss-ins-sim to simulate navigation performance of rocket simulations

## Environment
The sim is dependent on yaml, rocketpy, and the forked version of gnss-ins-sim   
These packages can be installed or, after installing Anacoonda, the following line can be run to set up a venv automatically:   
`conda env create --file <your rocket-nav directory>/config/environment.yml --name rocket-nav`   
Note: gnss-ins-sim must be installed manually. Clone and install using the following steps from the terminal:   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`git clone https://github.com/leapedcamera/gnss-ins-sim.git`   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`cd gnss-ins-sim`   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`git checkout tight`   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`python setup.py build`   
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`python setup.py install`   
This will install the local package to whatever python environment is activated

## Settings
The simulation works by generating a trajectory using `rocket.py`, and then performing navigation using `navigation.py`   
The RocketPy package is used to generate the trajectory, and all settings can be modified directly in `rocket.py`. Reference RocketyPy [documentation](https://docs.rocketpy.org/en/latest/user/first_simulation.html) for instructions.   
For the navigation, IMU parameters can be modified directly in `navigation.py`.    
The number of runs is specified in `sim.run(<number of runs>)`.   

## Running 
Run `rocket.py` to generate the trajectory, which will appear as `roocket.csv`.   
Run `navigation.py` to perform open-loop navigation on the trajectory.   
Position, velocity, and attitude error plots will appear both for the free inertial case and the GPS case.
