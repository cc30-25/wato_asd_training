#!/bin/bash

# This script generates a .env file to be used with docker-compose
# To override any of the variables in this script, create watod-config.sh 
#   in the same directory and populate it with variables
#   e.g. `COMPOSE_PROJECT_NAME=<NAME>`.

if [ -f /.dockerenv ]; then
	echo "Please run this in the host machine (not in the Docker container)"
	exit 1
fi

MONO_DIR="$(dirname "$(realpath "$0")")"
# moves us one level out to the root monorepo directory
MONO_DIR=${MONO_DIR%/*}

PROFILES_DIR="$MONO_DIR/profiles"

# Allow for local overrides of any of the below parameters
if [ -f "$MONO_DIR/watod-config.sh" ]; then
	source "$MONO_DIR/watod-config.sh"
fi

if ! [ -x "$(command -v git)" ]; then
    echo 'Error: git is not installed.' >&2
else
    BRANCH=${BRANCH:-$(git branch --show-current)}
fi

## ----------------------- Configuration (Subject to Override) ----------------------------
COMPOSE_PROJECT_NAME=${COMPOSE_PROJECT_NAME:-watod_$USER}

# Tag to use. Images as formatted as <IMAGE_NAME>:<TARGET_STAGE>-<TAG> with forward slashes replaced
# with dashes
TAG=$(echo ${TAG:-$BRANCH} | tr / -)
# replace / with -
TAG=${TAG/\//-}

# List of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - vis_tools     		  :   starts visualization tools (vnc and foxglove)
#   - production    		  :   configs for all containers required in production
#   - samples             :   starts sample ROS2 pubsub nodes
ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}

# List of profiles to IGNORE when using the --all flag
PROFILE_BLACKLIST=${PROFILE_BLACKLIST:-""}

## --------------------------- Images -------------------------------

# ROS2 C++ Samples
SAMPLES_CPP_AGGREGATOR_IMAGE=${SAMPLES_CPP_AGGREGATOR_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_aggregator"}
SAMPLES_CPP_PRODUCER_IMAGE=${SAMPLES_CPP_PRODUCER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_producer"}
SAMPLES_CPP_TRANSFORMER_IMAGE=${SAMPLES_CPP_TRANSFORMER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_transformer"}

# ROS2 Python Samples
SAMPLES_PYTHON_AGGREGATOR_IMAGE=${SAMPLES_PYTHON_AGGREGATOR_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_python_aggregator"}
SAMPLES_PYTHON_PRODUCER_IMAGE=${SAMPLES_PYTHON_PRODUCER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_python_producer"}
SAMPLES_PYTHON_TRANSFORMER_IMAGE=${SAMPLES_PYTHON_TRANSFORMER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_python_transformer"}

# ASD Training Images
GAZEBO_SERVER_IMAGE=${GAZEBO_SERVER_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_asd_training/gazebo_server"}
INFRASTRUCTURE_FOXGLOVE_IMAGE=${INFRASTRUCTURE_FOXGLOVE_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_asd_training/infrastructure_foxglove"}
ROBOT_OCCUPANCY_IMAGE=${ROBOT_OCCUPANCY_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/robot_occupancy"}
ROBOT_NAV_IMAGE=${ROBOT_NAV_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/robot_nav"}
ROBOT_CONTROL_IMAGE=${ROBOT_CONTROL_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/robot_control"}

## -------------------------- User ID -----------------------------

FIXUID=$(id -u) 
FIXGID=$(id -g) 

## --------------------------- Ports ------------------------------

BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
GUI_TOOLS_VNC_PORT=${GUI_TOOLS_VNC_PORT:-$((BASE_PORT++))}
FOXGLOVE_BRIDGE_PORT=${FOXGLOVE_BRIDGE_PORT:-$((BASE_PORT++))}
GAZEBO_PORT=${GAZEBO_PORT:-$((BASE_PORT++))}

## -------------------- Environment Variables -------------------------

# General
echo "# Auto-generated by ${BASH_SOURCE[0]}. Please do not edit." > "$PROFILES_DIR/.env"

echo "PROFILES_DIR=$PROFILES_DIR" >> "$PROFILES_DIR/.env"
echo "MONO_DIR=$MONO_DIR" >> "$PROFILES_DIR/.env"

echo "ACTIVE_PROFILES=\"$ACTIVE_PROFILES\"" >> "$PROFILES_DIR/.env"
echo "PROFILE_BLACKLIST=\"$PROFILE_BLACKLIST\"" >> "$PROFILES_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$PROFILES_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$PROFILES_DIR/.env"

echo "TAG=$TAG" >> "$PROFILES_DIR/.env"
echo "TARGET_STAGE=$TARGET_STAGE" >> "$PROFILES_DIR/.env"

echo "FIXUID=$FIXUID" >> "$PROFILES_DIR/.env"
echo "FIXGID=$FIXGID" >> "$PROFILES_DIR/.env"

# Ports
echo "BASE_PORT=$BASE_PORT" >> "$PROFILES_DIR/.env"
echo "FOXGLOVE_BRIDGE_PORT=$FOXGLOVE_BRIDGE_PORT" >> "$PROFILES_DIR/.env"
echo "GAZEBO_PORT=$GAZEBO_PORT" >> "$PROFILES_DIR/.env"

# ROS2 C++ Samples
echo "SAMPLES_CPP_AGGREGATOR_IMAGE=$SAMPLES_CPP_AGGREGATOR_IMAGE" >> "$PROFILES_DIR/.env"
echo "SAMPLES_CPP_PRODUCER_IMAGE=$SAMPLES_CPP_PRODUCER_IMAGE" >> "$PROFILES_DIR/.env"
echo "SAMPLES_CPP_TRANSFORMER_IMAGE=$SAMPLES_CPP_TRANSFORMER_IMAGE" >> "$PROFILES_DIR/.env"

# ROS2 Python Samples
echo "SAMPLES_PYTHON_AGGREGATOR_IMAGE=$SAMPLES_PYTHON_AGGREGATOR_IMAGE" >> "$PROFILES_DIR/.env"
echo "SAMPLES_PYTHON_PRODUCER_IMAGE=$SAMPLES_PYTHON_PRODUCER_IMAGE" >> "$PROFILES_DIR/.env"
echo "SAMPLES_PYTHON_TRANSFORMER_IMAGE=$SAMPLES_PYTHON_TRANSFORMER_IMAGE" >> "$PROFILES_DIR/.env"

# ASD Training Images
echo "GAZEBO_SERVER_IMAGE=$GAZEBO_SERVER_IMAGE" >> "$PROFILES_DIR/.env"
echo "INFRASTRUCTURE_FOXGLOVE_IMAGE=$INFRASTRUCTURE_FOXGLOVE_IMAGE" >> "$PROFILES_DIR/.env"
echo "ROBOT_OCCUPANCY_IMAGE=$ROBOT_OCCUPANCY_IMAGE" >> "$PROFILES_DIR/.env"
echo "ROBOT_NAV_IMAGE=$ROBOT_NAV_IMAGE" >> "$PROFILES_DIR/.env"
echo "ROBOT_CONTROL_IMAGE=$ROBOT_CONTROL_IMAGE" >> "$PROFILES_DIR/.env"

cat $PROFILES_DIR/.env