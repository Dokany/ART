# A.R.T | Automated rescue Team
A team composed of 5 robot cars, one patrolling car that autonomously monitors and detects emergencies; which can be medical emergencies, structure fires and acts of violence, and dispaches one or more of the respective emergency-handling robot cars; which can be an ambulance, a fire truck and/or a police car.

The patrolling car is equipped with a Jetson TX1 board, Pololu TReX Motor Controller, IMU connected through an Arduino board, and an additional USB camera to the Jetson's built in camera.

![alt text](https://lh3.googleusercontent.com/K4NFOGF_W8HdMejaXT-gIYBQM-Kk-eDXzCM9H4pH9UoosNK3w8Y5WZ-3_VJuq5fpbr46SiO_xEWw6a2It50WjPc5wJKY8UJgAEBITblfb7w8dnRhapsGBqrrERdTihg64ELRzHw_-JnWdk_pHSmY8KLQK4FIzgNX5p9VyS2lv3m1bQY9OO67Gh_2GahwfFVbJ2XtnnXGZWwd2aSOrT9fl93r3eJtSrjlRuE6o84hu93WPEgwOpqCsILf3pmSGnxePKc_TyJQyclSCS1CgY3gIuoqwB1iQrU21y6eQTC9vFtlmj6CgKZnXdiRDnX6tP-GfGyQSjy-XiVaX6zS2fTFwCoHO_flxERODufagusjH1fxQityjb5r-peLDHj17yqur2tf2pHWSKV1o3lmcqiiz7febHJO1C4hJ7eZpvJOnA3r5gzoCWSsS8zkD9yJq_BOXjTeCL5ZLDP0ODR-LFm4xza7pwdarBlJoqfssr784xhY4yKZ_Fq2yvy0IsKrVs8in0dTohNZ0hClOhecDgNub1fQbJOM_CLgvHb8cpyVc4frctDcxSDZa1NQaaPK-D0K_iYDg0ak1dK6UshFRhZySrqy8riuxi3OD_PrcXZVmNu3lasKGSpHCprXhWB3A0OLeXeFa6MAHRkjv0JsTZW4dRZB_ZNissrG=w1024-h768-no "Patrolling Car")

## How it Works


## Demo

## How We Built It
Using ROS, we created the auto_rescue package that runs several nodes responsible for handling different aspects of the project. The package includes the following nodes:
1. 

For the IMU, we uploaded MinIMU9.ino to the Arduino board connected to the Jetson via the ACM0 port.

## Installation Steps
   Install the [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation) distribution, then do the following:
```
 $ cd
 $ cd catkin_ws/src
 $ git clone https://github.com/Dokany/ART.git
 $ catkin_make
 $ source ./devel/setup.bash
```

## Dependencies
1. [GSCAM](http://wiki.ros.org/gscam)
2. [Jetson Csi Cam](https://github.com/peter-moran/jetson_csi_cam)
3. [PID](http://wiki.ros.org/pid)
    ```
    $ sudo apt-get install ros-kinetic-pid
    ```
    Then add the pid_code.launch file in the pid directory.

## Launching Steps
To launch the package's node, run the following:
```
    $ roslaunch auto_rescue
```

## Project Team
- [Aley Baracat](https://github.com/alybaracat)
- [John Sourour](https://github.com/johnsourour)
- [Karim Atwa](https://github.com/karimatwa)
- [Sara Seleem](https://github.com/saraseleem)
- [Yasmin ElDokany](https://github.com/Dokany)

#### Final Project submitted for the CSCE 432/4301 Embedded Systems course, at the American University in Cairo.
