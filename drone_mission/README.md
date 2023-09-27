# Drone_mission
THis repository contains mission execution python scripts for cps_challange_2020 gazebo simulation.
run final_mission_file.py is the execution file.
https://user-images.githubusercontent.com/97504177/236119033-6c1accd3-45f5-4fe9-8ea5-d87159a41130.mp4
![Studio_Project (2)](https://user-images.githubusercontent.com/97504177/236709060-351487da-2213-4cca-b190-518aad6e828a.gif)

## Instructions
Go to src folder of your workspace

`cd ~/catkin_ws/src`

Create rospy package

`catkin_create_pkg mission_py rospy`

Build new package

`cd ..`

`catkin_build`

`source devel/setup.bash`

Go to package folder

`roscd mission_py`

For python script files, create a folder named /scripts

`mkdir scripts`

`cd scripts`

copy the mission script file `drone_mission.py` to this folder.
chmod this file

`chmod +x final_mission_file.py`

Now you can execute this file through rosrun
But before that, run the cps_challange_2020 simulation and have the drone ready to take off!!

Run the file.

`rosrun mission_py final_mission_file.py`

Video
https://youtu.be/OF8OXZJcdMU



