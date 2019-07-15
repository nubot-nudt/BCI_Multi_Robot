#!/bin/bash
# Set the number of robots	
declare -i cyan_num=$(rosparam get /cyan/num)
declare -i magenta_num=$(rosparam get /magenta/num)
declare -i j                                   
# spawn robots
for ((i=0; i<cyan_num; i++))
do
    j=$i+1            
  
    rosrun nubot_control nubot_control_node nubot${j}  __name:=nubot${j}_control &
    PIDS[kill_num]=$!
    let "kill_num=kill_num+1"
    sleep 0.5        
done 

### kill thoes background processes
trap 'kill ${PIDS[*]}' SIGINT
wait
