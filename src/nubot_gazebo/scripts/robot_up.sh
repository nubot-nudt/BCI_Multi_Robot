#!/bin/bash

### Set the number of robots
declare -i cyan_num=$(rosparam get /cyan/num)
declare -i magenta_num=$(rosparam get /magenta/num)
declare -i j
football_name=$(rosparam get /football/name)
magenta_prefix=$(rosparam get /magenta/prefix)
cyan_prefix=$(rosparam get /cyan/prefix)
		                               
cyan_x=(-7 -7 -3 -3 -4.3)              # the first one is for goal-keeper
cyan_y=(-2  2 -3  3 -6.9)              # the first one is for goal-keeper 
magenta_x=(7  7  3  3)                 # the first one is not useful now
magenta_y=(-2 2 -3  3)                # the first one is not useful now-keeper

### spawn the football
rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/football/model.sdf -sdf \
                              -model ${football_name} \
                              -x 0.0 -y 0.0 -z 0.0 \
                               /

### spawn cyan robots
for ((i=0; i<cyan_num; ++i))
do
    j=$i+1                                              # skip the goal keeper
    rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/${cyan_prefix}${j}/model.sdf -sdf \
                                  -model ${cyan_prefix}${j} \
                                  -x ${cyan_x[$i]} -y ${cyan_y[$i]} -z 0.0 &
    sleep 0.5
done 

### spawn magenta robots
for ((i=0; i<magenta_num; ++i))
do
    j=$i+1                                              # skip the goal keeper
    rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/${magenta_prefix}${j}/model.sdf -sdf \
                                  -model ${magenta_prefix}${j} \
                                  -x ${magenta_x[$i]} -y ${magenta_y[$i]} -z 0.0 &
    sleep 0.5
done 

