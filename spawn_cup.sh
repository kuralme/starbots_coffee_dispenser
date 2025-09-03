#!/bin/bash
# Generate a random number between 1000 and 9999
RANDOM_ID=$(( RANDOM % 9000 + 1000 ))
ENTITY_NAME="cup${RANDOM_ID}"

# Path to the model
MODEL_PATH="./starbots_ur3e_gazebo/the_construct_office_gazebo/models/portable_cup_2/model.sdf"

# Spawn pose
X=14.1
Y=-18.2
Z=1.1
R=1.57
P=0
YAW=0

# Run the ros2 spawn command
ros2 run gazebo_ros spawn_entity.py \
    -file "${MODEL_PATH}" \
    -x "${X}" -y "${Y}" -z "${Z}" \
    -R "${R}" -P "${P}" -Y "${YAW}" \
    -entity "${ENTITY_NAME}"
