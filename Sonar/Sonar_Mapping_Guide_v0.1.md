# Sonar Mapping User Guide

## 1. Coordinate definition

### 1.1 Robot coordinate

Our robot is regraded as a 2D-point in physical world. We choose the front/active wheel and axle center as the robot's location. Robot coordinate is right-handed, X_robot is to the front, Y_robot points left and Z_robot points up. The front(x-axis) is 0-degree, counter-clockwise is positive, the left side ranges in [0deg, 180deg), clockwise is negative, the right side ranges in [-180deg, 0deg).


### 1.2 World coordinate

The origin of the world coordinate is the location of our bot when sonar mapping promagram starts, that is the initial front wheel and axle center. World coordinate is also right-handed and has the exactly same coordinate with robot coordinate, that is X_world is to the front, Y_world points left and Z_world points up. The initial front of our robot is 0-degree, and counter-clockwise is positive, the left side ranges in [0deg, 180deg), while clockwise is negative, the right side ranges in [-180deg, 0deg).


## 2. Definition of Robot pose and Sensor scan

### 2.1 Definition of Robot pose
In 2D world, we use three elements to define robot pose, (x, y, th).


### 2.2 Definition of Sensor scan
Since there are many sonars working at the same time, we use an vector to represent all sonars' scan data. For every single sonar's scan, we use four elements, (sx, sy, sth, len). (sx, sy, sth) is the sensor position and heading relative to robot center, and (len) is the detected distance of obstacle in current sensor scan.


## 3. The principle of sonar mapping 

### 3.1 Calculate line pass sensor and obstacle

Select sensor's location as start point, and obstacle's location plus a given expanded length as end point, assuming current robot pose as (_pose.x, _pose.y, _pose.th), current sonar scan's data as (_sonar.sx, _sonar.sy, _sonar.sth, _sonar.len), the start point, end point, obstacle point's location in world coordinate is as following,

```c
const double expandLen = 0.05 * 5;
startPointX = (_sonar.sx * cos(_pose.th) - _sonar.sy * sin(_pose.th)) + _pose.x;
startPointY = (_sonar.sx * sin(_pose.th) + _sonar.sy * cos(_pose.th)) + _pose.y;

endPointX = startPointX + (_sonar.len + expandLen) * cos(_pose.th + _sonar.th);
endPointY = startPointY + (_sonar.len + expandLen) * sin(_pose.th + _sonar.th);

obsPointX = startPointX + _sonar.len * cos(_pose.th + _sonar.th);
obsPointY = startPointY + _sonar.len * sin(_pose.th + _sonar.th);
```
```
Given two points formatting a line, we can calculate a line covers the crossing points.
```

### 3.2 Scoring each cell in the line using exponential value
For the cells across the line, we score each cell using exponential value according to the distance between the cell and obstacle cell. The nearer current cell closes to obstacle cell, the more the cell's value is close to 1. The further current cell is from obstacle cell, the more the cell's value is close to 0. If the current cell is obstacle cell, the the distance is 0 and the exponential value is 1, which means the cell's value is 1, 100% occupied. If the cell is unkown or not been explored yet, we set the exponential value to this cell. If the cell already has a value or been explored, we combine the current cell value and exponential value with a weight to calculate a new cell value.

### 3.3 result 
For the cells still not been explored yet, its value keeps 0.5. For those cells that have already been explored, it will get a value which is usually not 0.5. All the cells value generate a occupancy grid map.

## 4. About occupancy grid map

### 4.1 Introduction
Currently, we only support build square ogm(short for "occupancy grid map"), that is height and width are the same. For every cell's value, it should be located in [0.0, 1.0]. The initial value of all cells are 0.5, which means unknown or not been explored yet.

### 4.2 Tuning
You can define the ogm's resolution and square size according to your needs. The constructor of "OccupancyGridMap" class is *OccupancyGridMap(const double& res, const double& sq_size);*, the first parameter #res means the resolution, that is a single cell's size in meters, typical value is 0.05m, the second parameter #sq_size means the height or width in meters. For example, if res = 0.05m and sq_size = 80m, it means you to build an ogm, whose area is 80m * 80m = 6400 m^2, whose cells number is (80m / 0.05m) in height * (80m / 0.05m) in width = 2560000.

## 5. Dependancies

```
sudo apt-get install libgflags-dev libgoogle-glog-dev
```
## 6. Build and run

### 6.1 build test binary 
```c
- cd Map/Sonar
- mkdir build
- cd build
- cmake ..
- make
```
If you encouters Glog/GFlags issue, cp the Map/Sonar/3rdparty/FindGlog.cmake and Map/Sonar/3rdparty/FindGFlags.cmake to cmake modules folder, for example, cp Map/Sonar/3rd/FindGlog.cmake /usr/local/share/cmake-3.13/Modules/.

### 6.2 run test binary 

### 6.2.1 Setup command line
You can specify two gflags, *serialName* and *display* in command line. "/dev/ttyUSB1" is the default value for "serialName" and false is the default value for "display".

- Specify "serialName"
    * Change the USB port mode
        For example, the USB port for robot is "/dev/ttyUSB0", first, you need change its mode.
        - sudo chmod 666 /dev/ttyUSB0
    * Specify the right USB port
        For example, the USB port for robot is "/dev/ttyUSB0", then you need specify --serialName=/dev/ttyUSB0 in command line.

- Specify "display"
    If you connect robot to your own laptop, that is you have a displayer, then you can specify "display" as true, "--display=true", then you can view the online sonar mapping process. Since there is no displayer connected to miniPC, you have to sepecify "display" as false, that is "--display=false", in this case, you can only see a sonar_2d_ogm.txt after you exit the program and no visualization effect.
 
The whole running command is as following,
    ./test_sonar_mapping --serialName=/dev/ttyUSB1 --display=false
If you use "/dev/ttyUSB1" and there is no displayer, you can simply use ./test_sonar_mapping, which is exactly the same as the whole command.


### 6.2.1 Keyboard control
Basically, you can use "w/s/a/d" to control the robot. "w/s" controls linear velocity, add/subtract a step linear velocity, respectively, while "a/d" controls angular velocity, add/subtract a step angular velocity, respectively.

- step linear velocity: 0.05 m/s
- step angular velocity: 0.1 rad/s
- max linear velocity: 0.25 m/s
- max angular velocity: 1.5 rad/s

```
keys function:
  "w":
      adds a step linear velocity, reset the angular velocity to 0 at the same time.
  "s":
      subtract a step linear velocity, reset the angular velocity to 0 at the same time.
  "a":
      adds a step angular velocity, reset the linear velocity to 0 at the same time.
  "d":
      subtract a step angular velocity, reset the linear velocity to 0 at the same time.
  "z":
      stop the robot.
  "ESC" or "q":
      quit the program.
```

***Note1: Only if you press "ESC" or "q" to quit the program, a sonar_2d_ogm.txt will be generated, if you press CTRL+C, there is no "sonar_2d_ogm.txt".***

***Note2: You have better press "z" to stop the robot before you press "ESC" or "q" to quit the program, or even you quit the program, the robot is still moving and out of control unless you unplug the power supply.***

***Note3: Once the car is powered on, it can not be lifted off the ground, or the wheel will continue to rotate, you can not stop it from rotating even you restart the test program and press "z" trying to stop it, unless you unplug the power supply.***

## 7. Sonar Spec  

- Sonar name: HC-SR04
 
- Detected range: 2cm - 450cm
    * Blind zone: 2cm
    * max detected range: 450cm

- Detected angle: < 15 degree

- Resolution: 0.3cm

- Return value: only distance, no angle