#!/bin/bash

INSTALL_DIR=`pwd`

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get -y install \
    libjansson-dev \
    nodejs \
    npm \
    nodejs \
    libnode64 \
    libtinyxml-dev \
    mercurial \
    cmake \
    git


sudo apt-get update
sudo apt-get -y install locales curl gnupg2 lsb-release wget git
sudo locale-gen en_GB en_GB.UTF-8
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

#ENV LANG en_GB.UTF-8

sudo apt-get update && sudo apt-get -y install \
    pkg-config gazebo11 libgazebo11-dev vim wget gdb imagemagick python \
    ros-foxy-rcpputils ros-foxy-desktop ros-foxy-control-msgs ros-foxy-realtime-tools \
    ros-foxy-gazebo-ros ros-foxy-gazebo-plugins ros-foxy-gazebo-ros-pkgs ros-foxy-xacro \
    ros-robot-state-publisher ros-foxy-angles ros-foxy-teleop-twist-keyboard ros-foxy-rviz2 \
    ros-foxy-test-msgs ros-foxy-robot-localization \
    ros-foxy-py-trees-ros ros-foxy-py-trees-ros-tutorials ros-foxy-py-trees-ros-viewer \
    python3-argcomplete python3-vcstool python3-rosdep python3-colcon-common-extensions python3-pip \
    psmisc \
    google-mock \
	libceres-dev \
	liblua5.3-dev \
	libboost-dev \
	libboost-iostreams-dev \
	libprotobuf-dev \
	protobuf-compiler \
	libcairo2-dev \
	libpcl-dev \
	python3-sphinx



# Fetch and build the aruco library
cd $INSTALL_DIR
git clone https://siteks@bitbucket.org/siteks/aruco-3.0.0.git --depth 1
mkdir -p aruco-3.0.0/build
cd aruco-3.0.0/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install


# # Install gzweb
# #
# # Get the gzweb app, needs patching to work with gazebo11 but this fork is already patched
# cd $INSTALL_DIR
# git clone https://github.com/eurogroep/gzweb.git --depth 1 --branch gzweb_1.4.0-gazebo11
# cd gzweb
# #RUN git checkout gzweb_1.4.0-gazebo11
# # Prevent the side menu starting open, it uses up lots of space and is not very useful
# sed -i "s#globalEmitter.emit('openTab', 'mainMenu'#//globalEmitter.emit('openTab', 'mainMenu'#" gz3d/src/gzgui.js
# # Run deployment, this builds the gzweb system
# mkdir -p http/client
# sudo npm install
# node_modules/.bin/grunt build
# mkdir -p build
# cd build
# cmake ..
# make -j8
# mkdir ../gzbridge
# ../node_modules/.bin/node-gyp configure
# sed -i 's/Boost::/boost_/g' build/gzbridge.target.mk
# ../node_modules/.bin/node-gyp build -d 

# # Get the gazebo models and convert them. Don't download the full set,
# # its huge and we don't need any except minimal. But unless .gazebo
# # is present, Gazebo will insist on downloading..
# cd ..
# mkdir -p ~/.gazebo
# cp $INSTALL_DIR/dots_system/docker/gazebo_models ~/.gazebo/models
# #&&  git clone --depth 1 https://github.comV/osrf/gazebo_models.git /home/dots/.gazebo/models \
# #&&  rm -rf /home/dots/.gazebo/models/.git \
# cp -r ~/.gazebo/models http/client/assets \
# &&  ./webify_models_v2.py http/client/assets \
# # Copy in the standard gazebo materials and convert all textures
# # to png
# &&  cp -r /usr/share/gazebo-11/media /home/dots/gzweb/http/client/assets/ \
# &&  cd $INSTALL_DIR/gzweb/http/client/assets/media/materials/textures \
# &&  for f in *jpg; do convert $f ${f%.*}.png; done \

# Get version 2.1.5 of py_trees, which has proper mem/non-mem semantics. The
# version in the foxy apt packages is 2.1.2. Then remove the new setuptools and
# downgrade to one that doesn't give deprecatipn warnings. I'm sure there is a 
# better way of doing this..
sudo apt-get -y install python3-testresources
sudo rm -rf /opt/ros/foxy/lib/python3.8/site-packages/py_trees
sudo python3 -m pip install 'py_trees==2.1.5' -t /opt/ros/foxy/lib/python3.8/site-packages
# Downgrade setuptools to stop warnings from py_trees
sudo rm -rf /opt/ros/foxy/lib/python3.8/site-packages/py_trees/setuptools
sudo rm -rf /opt/ros/foxy/lib/python3.8/site-packages/setuptools
python3 -m pip install 'setuptools==54.0.0'

# Get and build modified copy of gazebo_ros_pkgs due to issues
# https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1247
# https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1211
# Without this hack to set the update rate of the /clock topic to 100Hz, from
# the default of 10Hz, various nodes such as robot_localization 
# do not run at the required rate
cd $INSTALL_DIR
mkdir -p gazebo_ros_pkgs_ws/src
cd gazebo_ros_pkgs_ws/src
git clone https://simonj23@bitbucket.org/simonj23/gazebo_ros_pkgs.git --branch foxy --depth 1
cd ..
source /opt/ros/foxy/setup.bash \
&&  colcon build --symlink-install --packages-select gazebo_dev gazebo_msgs gazebo_ros

# Get latest rosbag2 that will subscribe to all topics, regardless of qos settings.
# This version breaks te API and will not show up as a backport, but fixes many
# performance bugs and weird behaviours.
# This is forked off the official repo with a couple of build bugs fixed.
# https://github.com/ros2/rosbag2/issues/657
cd $INSTALL_DIR
sudo apt-get update
sudo apt-get -y install python3-pybind11 python3-json-tricks
mkdir -p rosbag2_ws/src
cd rosbag2_ws/src
git clone https://simonj23@bitbucket.org/simonj23/rosbag2.git --branch foxy-future --depth 1
cd ..
source /opt/ros/foxy/setup.bash \
&&  colcon build --merge-install


# Ensure ROS environment sourced and display available
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source $INSTALL_DIR/rosbag2_ws/install/setup.bash" >> ~/.bashrc
echo "source $INSTALL_DIR/gazebo_ros_pkgs_ws/install/setup.bash" >> ~/.bashrc






