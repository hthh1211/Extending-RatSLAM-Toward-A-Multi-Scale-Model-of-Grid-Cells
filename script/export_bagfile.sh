#!/bin/bash    

#while getopts f:t: option
#do
     #   case "${option}"
    #    in
   #             f) FILE=${OPTARG};;
  #              t) TOPIC_ROOT=${OPTARG};;
  #      esac
#done

set -x
echo "vt_id.dat"
rostopic echo  /turtlebot3/LocalView/Template > vt_id.dat
echo "em_id.dat"
rostopic echo  /turtlebot3/PoseCell/TopologicalAction_1 > em_id.dat
echo "pose.dat"
rostopic echo  /turtlebot3/ExperienceMap/RobotPose > pose.dat
echo "map.dat"
rostopic echo  /turtlebot3/ExperienceMap/Map > map.dat

