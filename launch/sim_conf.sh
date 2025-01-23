#!/bin/bash

scenario=${1}
scenario_type=${2}

laser_range=${3}
safety_factor=${4}
maxRobotLinearVelocity=${5}
maxRobotAngularVelocity=${6}

monitoring_type=${7}
navigation_type=${8}
occypancy_threshold=${9}
dynamic_areas_information=${10}

seed=${11}
maxObstLinearVelocity=${12}
maxObstAngularVelocity=${13}



###
dir_path=$(dirname "$(realpath "$0")")
dir_prev=$(dirname "$dir_path")



###
launch_filename=".launch"
scenario_name=""
if [ "$scenario" = "small" ]; then
    launch_filename="scen10$launch_filename"
    scenario_name="scen10"
elif [ "$scenario" = "large" ]; then
	launch_filename="scen7$launch_filename"
	scenario_name="scen7"
fi



###
world_filename="$dir_prev/world/${scenario_name}_${scenario_type}.world"
if [ ! -f "$world_filename" ]; then
    echo "world_filename $world_filename doesn't exist."
    exit 1
fi



###
obst_name="obstacle("
Nobst=0
while IFS= read -r line; do
    if [[ "$line" == *"$obst_name"* ]]; then
        Nobst=$((Nobst + 1))
    fi
done < "$world_filename"



###
obstacles_handling=-1
if [ "$dynamic_areas_information" = "memorize_all" ]; then
    obstacles_handling=0
elif [ "$dynamic_areas_information" = "forget_areas" ]; then
	obstacles_handling=1
elif [ "$dynamic_areas_information" = "forget_all" ]; then
	obstacles_handling=2
elif [ "$dynamic_areas_information" = "forget_los" ]; then
	obstacles_handling=3
else
	echo "NO ES NINGUNO"
fi

if [ ${obstacles_handling} -lt 0 ]; then
    echo "Value of dynamic_areas_information is incorrect."
    exit 2
fi




###
sed -i "s|<arg name=\"scenario_name\" value=\"[^\"]*\"/>|<arg name=\"scenario_name\" value=\"$scenario_name\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"scenario_type\" value=\"[^\"]*\"/>|<arg name=\"scenario_type\" value=\"$scenario_type\"/>|g" "$launch_filename"

sed -i "s|<arg name=\"Nobst\" value=\"[^\"]*\"/>|<arg name=\"Nobst\" value=\"$Nobst\"/>|g" "$launch_filename"

sed -i "s|<arg name=\"laser_range\" value=\"[^\"]*\"/>|<arg name=\"laser_range\" value=\"$laser_range\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"inflFact\" value=\"[^\"]*\"/>|<arg name=\"inflFact\" value=\"$safety_factor\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"linearVel\" value=\"[^\"]*\"/>|<arg name=\"linearVel\" value=\"$maxRobotLinearVelocity\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"angularVel\" value=\"[^\"]*\"/>|<arg name=\"angularVel\" value=\"$maxRobotAngularVelocity\"/>|g" "$launch_filename"

sed -i "s|<arg name=\"monitoring_type\" value=\"[^\"]*\"/>|<arg name=\"monitoring_type\" value=\"$monitoring_type\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"navigation_type\" value=\"[^\"]*\"/>|<arg name=\"navigation_type\" value=\"$navigation_type\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"occup_threshold\" value=\"[^\"]*\"/>|<arg name=\"occup_threshold\" value=\"$occypancy_threshold\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"forget_obstacles\" value=\"[^\"]*\"/>|<arg name=\"forget_obstacles\" value=\"$obstacles_handling\"/>|g" "$launch_filename"

sed -i "s|<arg name=\"seed\" value=\"[^\"]*\"/>|<arg name=\"seed\" value=\"$seed\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"obstLinearVel\" value=\"[^\"]*\"/>|<arg name=\"obstLinearVel\" value=\"$maxObstLinearVelocity\"/>|g" "$launch_filename"
sed -i "s|<arg name=\"obstAngularVel\" value=\"[^\"]*\"/>|<arg name=\"obstAngularVel\" value=\"$maxObstAngularVelocity\"/>|g" "$launch_filename"




###
roslaunch mada "$launch_filename"
