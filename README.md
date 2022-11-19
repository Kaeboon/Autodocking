# Autodocking
This is a project for my NUS undergraduate EG2605 UROP, to build an auto docking functionality to allow the SDR mobile robot provided by Solustar Pte Ltd to dock with charging station using visual input to detect AR tag and using PI-control to adjust wheel velocity. I uses ROS melodic for this project. The robot should start docking from 1.5 meter in front of the tag. The motivation is to let the robot complete all its navigation task, then return to this position, then start to dock to the charging station.

You may watch the actual SDR docking testruns here: [youtubelink](https://www.youtube.com/watch?v=4Hauzwl-VmU)
You may watch the the Gazebo simulation testrun here: [youtubelink](https://www.youtube.com/watch?v=QvYQjFvl8ig)

More details explanation of the methodology implementation, please see my final report [pdf](https://github.com/Kaeboon/Autodocking/blob/main/UROP_final_report.pdf)

For simulating the files in Gazebo:
You will need to download and build the following packages/application:
1) ar_track_alvar: [link](https://github.com/ros-perception/ar_track_alvar)
2) realsense-gazebo-plugin [link](https://github.com/pal-robotics/realsense_gazebo_plugin)
3) realsense-ros [link](https://github.com/IntelRealSense/realsense-ros)
4) SDR-simulation under this respiratory
And install this application:
1) realsense SDK 2.0 application [link](https://www.intelrealsense.com/sdk-2/)
And do the following:
1) Download the 'Gazebo models' from this respiratory and unzip.
2) Copy the 'willowgarage_with_artag.world' file to .gazebo/worlds of your computer.
3) Copy the 'artag123123' folder to .gazebo/model of your computer.
Then, to simulate it, just run this three launch files in order:
1) movingparts.launch (will launch gazebo. rviz)
2) pr2_indic_no_kinect_copy (will run artag detection node)
3) rotate_and dock (will run the auto docking operation)
You may be curious what other files do:
1) rotate_align_dock is an incomplete implementation of enabling robot to move to the opposite of artag before dock [youtubelink](https://www.youtube.com/watch?v=ojB2HuYRQBQ), feel free to work on it.


For running the docking files in the actual SDR:
You just need to download and build the following packages in the actual SDR:
1) ar_track_alvar: [link](https://github.com/ros-perception/ar_track_alvar)
2) auto_docking under this respitatory
Then, to run the acutodocking operation just:
1) pr2_indic_no_kinect_copy (will run artag detection node)
2) rotate_and dock (will run the auto docking operation)
