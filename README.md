# senior-design-2019
Repo for ECE Senior Design code

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