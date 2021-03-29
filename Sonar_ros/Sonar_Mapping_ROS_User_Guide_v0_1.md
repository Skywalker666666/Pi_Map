# Sonar Mapping ROS User Guide v0.1

## 1. Introduction
This doc introduces how to obtain indoor map via sonars. Sonar mapping method (Stand-alone version) via a DIY chassis has been discussed in another user guide. Here, several sonar mapping methods running upon ROS are described, and methods in this doc utilizes Pioneer 3DX chassis.

## 2. Installation
The installation can refer to [How to use ROSARIA](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA).

### 2.1 Install MobileRobots ARIA Ubuntu Package
To install the latest version of ARIA library, we can download it from [ARIA website](http://robots.mobilerobots.com/wiki/Aria).
But this website is dead.

 If the robots.mobilerobots.com website or the ARIA download are no longer available, then a modified fork of ARIA is also available by cloning or forking [AriaCoda](http://github.com/reedhedges/AriaCoda).

So I git cloned reedhedges/AriaCoda, this is aria library. (Be careful, AriaCoda is still evolving now. There is new commits, basically the version we use is: **hash #: fd608e5962108fd3c809ee852a313f072b9665be**)
Then
```
ubuntu@ubuntu-space:~/Documents/Tools/AriaCoda_alternative/AriaCoda
Then 
make clean
make -j4
sudo make install
echo /usr/local/Aria > /etc/Aria
```

`ARIA has been installed in /usr/local/Aria.`

To be able to use the ARIA libraries, you must now add /usr/local/Aria/lib
to your LD_LIBRARY_PATH environment variable, or to the /etc/ld.so.conf system file,
then run 'ldconfig'

```
ubuntu@ubuntu-space:~$ vi ~/.bashrc
```
edit the bash file.
```
LD_LIBRARY_PATH="/usr/local/Aria/lib:$LD_LIBRARY_PATH"
```
```
ubuntu@ubuntu-space:~/Documents/Tools/AriaCoda_alternative/AriaCoda$ sudo ldconfig
```

### 2.2 Bring ROSARIA into the workspace
Here, `rosaria` source code is copied from 'Map/Sonar_ros/rosaria' at Map github repository, and put it in ~/catkin_ws/src.

If you want to know where it is from, the original source code of amor-ros-pkg/rosaria can be downloaded from [amor-ros-pkg/rosaria](
https://github.com/amor-ros-pkg/rosaria).
The commit we are using now is **Hash #: 44a05dc2a3c7552dd4925c40d7a3fe802f82ffcb**.
```
cd ~/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
git checkout hashvaluexxxx
```

For src code used in our project, need to modify one line in *RosAria.cpp*:
change
669       p.z = 0.0;
to
670       p.z = reading->getRange() / 1000.0;

### 2.3 Bring ROSARIA_client into the workspace
Copy `rosaria_client` source code from from 'Map/Sonar_ros/rosaria_client' at Map github repository to ~/catkin_ws/src/
Same here, original source code is downloaded it from [pengtang/rosaria_client](https://github.com/pengtang/rosaria_client). Then, we design our Sonar mapping algorithm within these files.

### 2.4 Compilation
The compilation of `rosaria` and `rosaria_client` is combined with Lidar related source file, because all of them are put in same work space.
```
catkin_make_isolated --install --use-ninja -j4
```
"--force-cmake" option might be needed if error occurs.


## 3. Running
Same to Lidar mapping. The mapping procedure of Sonar can be ran either Online or Offline.

### 3.1 Online Mode
#### 3.1.1 Launch sonar mapping
Open a terminal on your PC, ssh into controller computer, plug in USB cable of Pioneer 3Dx (12V DC power supply), assign sudo privilege. Then navigate to
```
$~/Documents/Tools/catkin_ws
```
Then
```
source install_isolated/setup.sh
source devel_isolated/setup.sh
```
Then

```
ubuntu@ubuntu-space:~/Documents/Tools/catkin_ws$  roslaunch rosaria_client rosaria_client_sonar_launcher.launch 
```
#### 3.1.2 Keyboard Control
use "left arrow/right arrow/up arrow/down arrow/a/z/s/x" to control the robot. (code is based on virtual key code)

**left arrow**: 
turn left (angular speed = pos current angular speed;  linear speed = 0)
**right arrow**: turn right (angular speed = neg current angular speed;  linear speed = 0)
**up arrow**: forward (angular speed = 0;  linear speed = pos current linear speed)
**down arrow**: reverse (angular speed = 0;  linear speed = neg current linear speed)
**a**: increase linear speed
**z**: decrease linear speed
**s**: increase angular speed
**x**: decrease angular speed
Then, you can controll the robot to visit different places.

If you want to finish mapping, just press **q**, wait a minute. Press "Ctrl + c" after the Sonar raw map .txt file is successfully saved.

Because Sonar mapping need pose information from LiDAR mapping, so we will launch LiDAR mapping terminals first, then launch the terminal for Sonar.

One more thing, you can modify the output path of sonar file in "sonar_mapping_3dx.cpp".

#### 3.1.3 Rename output files
Same to LiDAR, 
Rename output files by user-defined name string.
```
./rename_lidar_sonar_out_files.sh 0819_A4_F2_08_V2_LJ_offline
```
*rename_lidar_sonar_out_files.sh* can be donwloaded from "Map/Fusion/" folder of github repository.

### 3.2 Offline Mode
#### 3.2.1 Launch offline sonar mapping
Open a terminal, then navigate to
```
$~/Documents/Tools/catkin_ws
```
Then
```
source install_isolated/setup.sh
source devel_isolated/setup.sh
```
Then download sonar scan raw data file and pose raw data file from Google drive [link](https://drive.google.com/drive/folders/1hidey2XmoEzkyYZ41_HIcDlyi4odxClL?usp=sharing). 8 test cases are available at Pi_LiDAR_Sonar_Dataset/Sonar_Data/.


```
TAG_NAME="0807_SZU_L6F02_1m3_08"
VERSION="0827_SZU_L6F02_08_V4_LJ_offline"

###### "LPF": local pose,
###### "PGPF": partial global pose, 
###### "FGPF": final global pose,
###### "FGPFINTP": final global pose interpolation. Extra file will be gen

###### "CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
###### "CoD" -- "DIRECT_CONE_ALGORITHM"
###### "Bay" -- "BAYESIAN_ALGORITHM"
###### "DS"  -- "DS_ALGORITHM"

rosrun rosaria_client sonar_mapping_3dx_offline -scan_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_scan_raw_data_sonar.txt" -pose_raw_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${TAG_NAME}_pose_raw_data.txt" -sonar_2d_ogm_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/offline_results/${VERSION}_sonar_2d_ogm_offline_FGPFINTP_Bay.txt"  -orig_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Orig_LocPo_TX.txt"  -opti_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Opti_GloPo_TX.txt" -final_node_pose_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_TX.txt" -pose_type="FGPFINTP" -sonar_map_algo_option="Bay" -final_node_pose_interpo_path="/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/${VERSION}_Final_GloPo_Interpo.txt"

```
If you want to finish mapping, just press "Ctrl + c" after the Sonar raw map .txt file is successfully saved.

One more thing, you can modify the output path of sonar file in "sonar_mapping_3dx.cpp".

#### 3.2.2 Rename output files
Same to LiDAR, 
Rename output files by user-defined name string.
```
./rename_lidar_sonar_out_files.sh 0819_A4_F2_08_V2_LJ_offline
```
*rename_lidar_sonar_out_files.sh* can be donwloaded from "Map/Fusion/" folder of github repository.


## 4. Algorithms
"CLD" -- "DIRECT_CENTRAL_LINE_ALGORITHM"
"CoD" -- "DIRECT_CONE_ALGORITHM"
"Bay" -- "BAYESIAN_ALGORITHM"
"DS"  -- "DS_ALGORITHM"



