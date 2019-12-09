# senior-design-2019
Repository for ECE Senior Design code

# Autonomy
All of the code, including dependencies, are in `catkin_ws`. The autonomy assumes ROS Kinetic with Ubuntu 16.04; however, the system builds and runs as is in 18.04 with ROS Melodic. 

To install the packages, run `scripts/install.sh` to pull all dependencies and build the workspace. Additionally, to run the stack, you need to install the DJI ROS Onboard SDK and generate an app key and app id with your DJI account. When running the SDK with the drone, you need to tell the drone that it is okay that the USB is still connected. This is done by running `scripts/enable_usb.sh`.

Inside of the workspace, the two main packages are `mbz2020_common` and `mbz2020_planner`. Common has the launch files, random scripts, and ROS message file definitions needed to get the whole system working. Inside the planner package are some dependencies, `mbz2020_target_planners`, and `robot_motions_server_ros`. 

`robot_motions_server_ros` is the framework that wraps ROS ActionLib and allows for generic tasks. Inside `mbz2020_target_planners` are nodes for trajectory generation and the task config, where the pickup task/action is defined (i.e. `mbz2020_target_planners/src/task_config/tasks/pickup_target.py`). 

# Ball detection
The tools directory was used to label datasets, change path names, and convert
file types.

Tools - Video labeling tool was a gui utilized to manually draw bounding boxes
for labeling dataset. Label_directory_switch was used to change the path of each
xml label. Train_test_switcharoni is used to randomize the data which is used to
train vs the data used to test the neural network. Xml to csv was uzed to produce
a file containing all the labelled data.

Object Detection - A package on github with slight modification. This is part of
the tensorflow research package. The neural network uses this to train and run the
neural network's inference graph.

object_detection_with_own_model - The code needed to run a visual test of the neural
network. This also includes the localization algorithm.

The neural network inference graphs are in the google drive linked with this project.
