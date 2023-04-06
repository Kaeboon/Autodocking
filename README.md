# Autodocking
This is a project for my NUS undergraduate EG2605 UROP, to build an auto docking functionality to allow the SDR mobile robot provided by Solustar Pte Ltd to dock with charging station using visual input to detect AR tag and using PI-control to adjust wheel velocity. The robot should start docking from 1.5 meter in front of the tag. The motivation is to let the robot complete all its navigation task, then return to this position, then start to dock to the charging station.

You may watch the actual SDR docking testruns here: [youtubelink](https://www.youtube.com/watch?v=4Hauzwl-VmU)

You may watch the the Gazebo simulation testrun here: [youtubelink](https://www.youtube.com/watch?v=QvYQjFvl8ig)

This project is developed using Ubuntu 18.04 and ROS1 Melodic. The ARTag used is ID:8 and size 9cm x 9cm. The back realsense camera is used for visual input. For more detailed explanation of the methodology implementation, please refer to my final report [pdf](https://github.com/Kaeboon/Autodocking/blob/main/UROP_final_report.pdf)

--------------------------------------------------------------------------------------------


For simulating the files in Gazebo:

Prereq: Ubuntu and ROS1 installed.

You will need to install this application by follwoing the guide:
1) realsense SDK 2.0 application [link](https://www.intelrealsense.com/sdk-2/)

You will need to download and build the following packages in your workspace.
1) ar_track_alvar: [link](https://github.com/ros-perception/ar_track_alvar)
2) realsense-gazebo-plugin [link](https://github.com/pal-robotics/realsense_gazebo_plugin)
3) realsense-ros [link](https://github.com/IntelRealSense/realsense-ros)
4) SDR-simulation under this respiratory

You will need to check your ~/.gazebo folder for 'worlds' and 'models' folders. If they are not there, do the followings:
1) download from https://github.com/leonhartyao/gazebo_models_worlds_collection and copy both folders there, especially the willowgarage.world + sun, ground models.
2) download from https://github.com/osrf/gazebo_models and copy the additional models there. especially willowgarage models.

And do the following:
1) Download the 'Gazebo models' from this respiratory and unzip.
2) Copy the 'willowgarage_with_artag.world' file to .gazebo/worlds of your computer.
3) Copy the 'artag123123' folder to .gazebo/model of your computer.

Then, to simulate it, just run this three launch files in order with roslaunch:
1) movingparts.launch (will launch gazebo with model and rviz)
2) pr2_indic_no_kinect_copy.launch (will run artag detection node)
3) rotate_and_dock.launch (will run the auto docking operation)

You may be curious what other files do:
1) rotate_align_dock is an incomplete implementation of enabling robot to move to the opposite of artag before dock, as shown here: [youtubelink](https://www.youtube.com/watch?v=ojB2HuYRQBQ), feel free to work on it.


----------------------------------------------------------------------------------------

For running the docking files in the actual SDR Robot (from Solustar):
You just need to download and build the following packages in the SDR's NUC:
1) ar_track_alvar: [link](https://github.com/ros-perception/ar_track_alvar)
2) auto_docking under this respitatory

Then, to run the autodocking operation, just do the following in order:
1) (initialize the SDR robot to start the wheel and realsense)
2) roslaunch pr2_indic_no_kinect_copy.launch (will run artag detection node)
3) roslaunch rotate_and dock.launch (will start running the auto docking operation)

-------------------------------------------------------------------------------------------

The UROP Presentation (1).pdf file is outdated and should not be used as guide.

If you need any more information or help, feel free to contact me~
