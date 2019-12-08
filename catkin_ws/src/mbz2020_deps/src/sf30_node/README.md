##This is a ROS node for [SF30 rangefinders*](http://www.lightware.co.za/shop/en/4-drone-altimeters) ##
(*it is probably compatible with other models: SF02, SF10, SF11 - not tested!)

The node assume the following setup for the laser (I used the [Lightware](http://www.lightware.co.za/shop/en/content/8-software) terminal from the manufacturer):

  
```
#!term

   1: Active data port           USB distance in m
   2: Resolution                 0.03 m
   3: Serial port update rate    1000 / sec  (actual = 1665 / sec)
   4: Serial port baud rate      115200
   5: Analog port update rate    1 / sec  (actual = 1 / sec)
   6: Analog maximum range       256 m
   7: Alarm activation distance  17.50 m
   8: Alarm latch                Off
   9: USB port update rate       50 / sec  (actual = 50 / sec)

```

Before you leave the terminal, make sure the sensor is sending messages of the form (hit space to make it happen):


```
#!term
0.57 m
0.57 m
0.59 m
0.57 m
0.59 m
0.59 m
0.55 m
0.59 m
0.57 m
0.57 m
0.57 m

```


###To install the package: ###

Create a catkin workspace. For instructions on how to create the workspace go [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Download and compile the package:


```
#!bash

cd catkin_ws/src
git clone git@bitbucket.org:castacks/sf30_node.git
cd ..
catkin_make
```


###To run the node: ###

In a terminal run:

```
#!bash
source devel/setup.bash
roslaunch sf30_node sf30.launch


```

###Parameters: ###
There are a number of parameters in `sf30.launch` that control the behavior of this driver. Please read these carefully and modify as needed. 

`portname`: name of the serial port the SF30 is connected on. Defaults to `/dev/ttyUSB0`.
`topic_name`: name of topic to publish to, default is `range`.
`frame_id`: name of the frame used when publishing message. 
`publish_as_pose`: if `true`, the range will be published as a `geometry_msgs::PoseWithCovarianceStamped`. If `false`, the message will be of type `sensor_msgs::LaserScan`. When published as a `LaserScan`, the intensities field on this message means data confidence. It is `1` if we can trust the given range. Defaults to `true`. 
`debug`: print debug information when `true`
`update_rate`: update rate of node. Defaults to 50Hz. 
`altitude_variance`: variance to use when publishing range as a pose. Default is .09. 


### Who do I talk to? ###

* Guilherme Pereira - gpereira@ufmg.br

### Final Remarks ###

* The laser sensor uses an internal FTDI serial-to-USB converter. Therefore, your Linux system must provide support for this device. Most of distributions come with this support but some embedded distributions need to be set. [Here](http://elinux.org/Jetson/Tutorials/Program_An_Arduino) is a good tutorial on how to set the FTDI support on Jetson TK1 systems. It was tested on the DJI's Manifold.

* Some embedded Linux distributions suspend their inactive USB ports to save power. This may cause the sensor to stop working after the first use. [Here](http://jetsonhacks.com/2015/05/27/usb-autosuspend-nvidia-jetson-tk1/) is how to disable the auto-suspend function to prevent problems.

* FTDI devices create a file in the folder `/dev/serial/by-id/` that can be used as a unique identifier for the device. To use this identifier, replace the parameter `portname` in the file `sf30.launch` from `/dev/ttyUSB0` to `/dev/serial/by-id/ID_OF_YOUR_DEVICE`. This is very useful when several devices are connected to the same computer.

### License ###
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
