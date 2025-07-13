# Quick Start

## installation
We tested our code on Ubuntu20.04 with ROS noetic.

1. Clone the code from github
    ```
    mkdir ws_benchRNR && cd ws_benchRNR
    git clone https://github.com/sjtu-cyberc3/BenchRNR src
    ```

2. Install the required ROS packages
    ```
    sudo apt update
    sudo apt install -y \
    ros-${ROS_DISTRO}-roscpp \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-roslib \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-message-generation \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-catkin
    ```

3. Install the following dependencies
    ```
    sudo apt install -y \
    libpcl-dev \
    libopencv-dev \
    libceres-dev \
    libproj-dev \
    cmake
    ``` 

4. Livox ROS Driver

    Our packages depend on livox_ros_driver, which must be installed and built first. Please follow [livox ros driver installation](https://github.com/Livox-SDK/livox_ros_driver).

    Remember to set your `livox_ros_driver_DIR` in the two CMakelists.txt.
    ```
    set(livox_ros_driver_DIR "/your/path/to/livox_ros_driver/devel/share/livox_ros_driver/cmake")
    ```

5. Install PROJ

    We used the PROJ library for UTM coordinate transformation. Please install PROJ-7.0.0 to avoid troubles.
    ```
    curl -LO https://download.osgeo.org/proj/proj-7.0.0.tar.gz
    tar -xzvf proj-7.0.0.tar.gz
    cd proj-7.0.0
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/proj-7.0.0
    make -j$(nproc)
    sudo make install
    ```
## Build & Run
build the project

```
cd ws_benchRNR

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

run the project by:
```
# Open Terminal 2
# launch
roslaunch clustering_and_tracking localization.launch arg_traj_num_str:="1" arg_model_init:=1

# Open Terminal 2
# play the rosbag
rosbag play rosbag1.bag --clock

# Open Terminal 3
# Specify the vehicle to track via terminal. `35` is the id of the vehicle to track.
rostopic pub -1 /change_id_topic std_msgs/Int32 35

```

This launch file accepts multiple parameters. The following explains the meaning of each parameter：

1.  `arg_traj_num_str`： the index of the rosbag being played.
2.  `arg_model_init`: decides where the model is initialized. `1` is the right entrace, `2` is the left entrace.
3. `rviz`: whether use rviz to visualize.
4. `arg_auto`: set it to be true so that the localization system starts and ends automatically.

If to use multi-line LiDAR:

build the project with `-DUSE_MULTI=ON`：
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_MULTI=ON
```

1. set `arg_use_multi` to True
2. set `arg_rings` to `128` `64` or `32` to change the number of lines.

    For example:
    ```
    roslaunch clustering_and_tracking localization.launch arg_traj_num_str:="3" arg_model_init:=2 arg_use_multi:=1 arg_rings:=32
    ```


## Auto run & test

1. Change `BAG_DIR` to your actual directory in [run_all_trajs.sh](run_all_trajs.sh)
2. Run the script:
    ```
    ./run_all_trajs.sh --use-multi true --ring-num 128 --use-rviz true
    ```

    `[--use-multi true|false]` decides whether to use the multi-line LiDAR (remember to rebuild the project with `-DUSE_MULTI=ON`), if it is set false, it will use nn-repetitive scanning.

    `[--ring-num 128|64|32]` decides how many lines the multi-line LiDAR will be.

    `[--use-rviz true|false]` decides whether to visualize with rviz.
3. Run the following to get results:
    ```
    cd src/clustering_and_tracking/data/scripts
    python combine_txt.py & test_exp_table.py
    ```

