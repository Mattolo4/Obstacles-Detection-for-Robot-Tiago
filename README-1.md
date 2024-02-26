GROUP 12
Gabriel Taormina, gabriel.taormina@studenti.unipd.it
Stefano Trenti, stefano.trenti@studenti.unipd.it,
Matteo Villani, matteo.villani@studenti.unipd.it,


Repo link: https://bitbucket.org/intelligent-robotics-12/ir2324_group_12/src/master/
Google Drive folder: https://drive.google.com/drive/u/1/folders/1Npb3Vff7EKpSnvJJbKma_z7jBRwlIzMS

## Compiling istructions
You will need 4 terminal sourced in ~/catkin_ws

For terminal 1-2-3-4 run:

> start_tiago
> source /opt/ros/noetic/setup.bash 
> source /tiago_public_ws/devel/setup.bash

> catkin build  (one terminal is fine)
> source ~/catkin_ws/devel/setup.bash

##Running instructions

For terminal 1:
> roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library
(wait for the simulation started)

For terminal 2:
> roslaunch tiago_iaslab_simulation navigation.launch
(wait for prompted 'odom recieved')

For terminal 3:
> rosrun Assignment_1_group_12 server

For terminal 4:
> rosrun Assignment_1_group_12 client <x> <y> <yaw_angle>