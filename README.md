## Dots development environment

This is the Dots development environment. It is based on Docker containers, so it can run on Windows, Linux, and OSX. The directory where the environment is started is mounted within the environment so all changes are persistant.

### Prerequisites
- Working Docker installation
- (On windows) Windows System for Linux (WSL)
- git and make

### Installation
To download and start the environment, go to the the directory you want to keep your work, and do:
```
git clone --recursive git@bitbucket.org:hauertlab/dots_system.git
cd dots_system
make
```
This will build the docker images then use docker-compose to start it running. The build process will take some time the first time it is done.

Then do:
```
make run
```
to start the docker environment up.

The environment presents two interfaces, both web-based. The [VSCode interface](http://localhost:8080/?workspace=/home/dots/dots_system/dots.code-workspace) is a web version of the VSCode editor. This is where you will do most coding. Click the link and you should see something like:![](images/vscode.jpg)


The [Linux desktop](http://localhost:8081/) is a web-based VNC view into a standard Linux desktop. Graphical applications such as Gazebo and Rviz will appear here. The size of the desktop is based on the size of the browser window when you open the link - make a separate browser window the size you want before opening the link.

To make sure everything is working, open a terminal in the vscode window by going Menu (top left)->Terminal->New terminal. This places you in a ROS2 workspace with the various packages for the simulator and example controller below the ```src``` directory. The simplified package structure is:
```
src
└── dots_gazebo
    ├── dots_example_controller     Example python controller performing random walk
    ├── dots_sim                    Gazebo simulator
    ├── dots_sim_support            Support files for the simulator
    └── gazebo_plugins              Simulator plugins for lifting mechanism, the coloured leds and motor drive
```
To build the packages, do:
```
colcon build --symlink-install
source install/setup.bash
```
This will traverse the packages in the ```src``` directory, compile C++ code, then install C++ and Python executables and support files. These are placed in the ```install``` directory. The ```source``` command makes the just installed packages visible to the ROS system. To run an example, do:
```
ros2 launch dots_sim run_2_explore.launch.py
```
This will start the Gazebo simulator GUI in the Linux desktop, then spawn two robots at different locations within the simulated world. It should look like this:
![](images/gazebo.jpg)


The simulator starts up paused, press play to start it running. The two robots should move in random curved trajectories, changing direction when they encounter an obstacle.

### Developing
The general development flow is to make changes to your code, do ```colcon build --symlink-install```, then try out your changes. Its only necessary to do ```source install/setup.bash``` in a new terminal or after building for the first time. If you make changes to Python code, it is not usually necessary to do ```colcon build``` since symbolic links are installed for interpreted code, but sometimes this doesn't seem to work.

It is possible to start various parts separately, you might often want to e.g. have the simulator running while manually starting robots.
Some possible examples, launch with the Rviz2 app to visualise robot sense data:
```
ros2 launch dots_sim run_2_explore.launch.py rviz:=true
```
Just launch gazebo:
```
ros2 launch dots_sim gazebo.launch.py
```
Launch a single robot without a controller at a location within the arena (requires Gazebo to be running)
```
ros2 launch dots_example_controller basic.launch.py robot_name:=r1 robot_pose:=-0.2,0.3,1.57
```


### Issues
Sometimes the linux desktop does not correctly size to the window size. Only fix so far is to ctrl-c the docker session and restart with `make run`.


Sometimes the Gazebo simulator doesn't corectly stop when  ctrl-c'd. A new simulation won't start because another copy is already running, there will be an error message like `EXCEPTION: Unable to start server[bind: Address already in use]. There is probably another Gazebo process running`. Fix, do `killall gzserver gzclient` before starting new simulation.
