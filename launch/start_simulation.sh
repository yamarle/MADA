#!/bin/bash

# Scenario variables
scenario="small" # (small, large)
scenario_type="nondense" # (nondense, dense)

# Robot variables
laser_range="10"
safety_factor="1.4"
maxRobotLinearVelocity="0.55"
maxRobotAngularVelocity="1.0"

# Planner variables
monitoring_type="mada" # (greedy, madp, mada)
navigation_type="pf" # (pf, dwa)
occypancy_threshold="1.0" # values from 0 to 1 → (0: non-traversable areas; values 0-1: ; 1: areas with obstacles will be avoided if other paths do not exist)
dynamic_areas_information="forget_all" # (memorize_all, forget_areas, forget_all, forget_los)

# Obstacle variables
seed="0" # (0: random, >0: selected seed)
maxObstLinearVelocity="0.2"
maxObstAngularVelocity="0.5"



# Configure and launch simulation
./sim_conf.sh $scenario $scenario_type $laser_range $safety_factor $maxRobotLinearVelocity $maxRobotAngularVelocity $monitoring_type $navigation_type $occypancy_threshold $dynamic_areas_information $seed $maxObstLinearVelocity $maxObstAngularVelocity
