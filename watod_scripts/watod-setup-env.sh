#!/bin/bash
set -e

# This script generates a .env file to be used with docker-compose
# To override any of the variables in this script, create watod-config.sh 
#   in the same directory and populate it with variables
#   e.g. `COMPOSE_PROJECT_NAME=<NAME>`.

if [ -f /.dockerenv ]; then
	echo "Please run this in the host machine (not in the Docker container)"
	exit 1
fi

# Retrieve git branch
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
ACTIVE_MODULES=${ACTIVE_MODULES:-""}

# Docker Registry to pull/push images
REGISTRY_URL=${REGISTRY_URL:-"ghcr.io/watonomous/wato_monorepo"}

REGISTRY=$(echo "$REGISTRY_URL" | sed 's|^\(.*\)/.*$|\1|')
REPOSITORY=$(echo "$REGISTRY_URL" | sed 's|^.*/\(.*\)$|\1|')

## --------------------------- Images -------------------------------
# NOTE: ALL IMAGE NAMES MUCH BE IN THE FORMAT OF <COMPOSE_FILE>_<SERVICE>

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
ROBOT_COSTMAP_IMAGE=${ROBOT_COSTMAP_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/robot_costmap"}
ROBOT_NAV_IMAGE=${ROBOT_NAV_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/robot_nav"}
ROBOT_CONTROL_IMAGE=${ROBOT_CONTROL_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/robot_control"}

## --------------------------- Ports ------------------------------

BASE_PORT=${BASE_PORT:-$(($(id -u)*20))}
FOXGLOVE_BRIDGE_PORT=${FOXGLOVE_BRIDGE_PORT:-$((BASE_PORT++))}
GAZEBO_PORT=${GAZEBO_PORT:-$((BASE_PORT++))}

## -------------------- Environment Variables -------------------------

# General
echo "$MODULES_DIR"
echo "# Auto-generated by ${BASH_SOURCE[0]}. Edit at own risk." > "$MODULES_DIR/.env"

echo "MODULES_DIR=$MODULES_DIR" >> "$MODULES_DIR/.env"
echo "MONO_DIR=$MONO_DIR" >> "$MODULES_DIR/.env"

echo "ACTIVE_MODULES=\"$ACTIVE_MODULES\"" >> "$MODULES_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$MODULES_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$MODULES_DIR/.env"

echo "TAG=$TAG" >> "$MODULES_DIR/.env"

# Ports
echo "BASE_PORT=$BASE_PORT" >> "$MODULES_DIR/.env"
echo "FOXGLOVE_BRIDGE_PORT=$FOXGLOVE_BRIDGE_PORT" >> "$MODULES_DIR/.env"
echo "GAZEBO_PORT=$GAZEBO_PORT" >> "$MODULES_DIR/.env"

# ROS2 C++ Samples
echo "SAMPLES_CPP_AGGREGATOR_IMAGE=$SAMPLES_CPP_AGGREGATOR_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_CPP_PRODUCER_IMAGE=$SAMPLES_CPP_PRODUCER_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_CPP_TRANSFORMER_IMAGE=$SAMPLES_CPP_TRANSFORMER_IMAGE" >> "$MODULES_DIR/.env"

# ROS2 Python Samples
echo "SAMPLES_PYTHON_AGGREGATOR_IMAGE=$SAMPLES_PYTHON_AGGREGATOR_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_PYTHON_PRODUCER_IMAGE=$SAMPLES_PYTHON_PRODUCER_IMAGE" >> "$MODULES_DIR/.env"
echo "SAMPLES_PYTHON_TRANSFORMER_IMAGE=$SAMPLES_PYTHON_TRANSFORMER_IMAGE" >> "$MODULES_DIR/.env"

# ASD Training Images
echo "GAZEBO_SERVER_IMAGE=$GAZEBO_SERVER_IMAGE" >> "$MODULES_DIR/.env"
echo "INFRASTRUCTURE_FOXGLOVE_IMAGE=$INFRASTRUCTURE_FOXGLOVE_IMAGE" >> "$MODULES_DIR/.env"
echo "ROBOT_COSTMAP_IMAGE=$ROBOT_COSTMAP_IMAGE" >> "$MODULES_DIR/.env"
echo "ROBOT_NAV_IMAGE=$ROBOT_NAV_IMAGE" >> "$MODULES_DIR/.env"
echo "ROBOT_CONTROL_IMAGE=$ROBOT_CONTROL_IMAGE" >> "$MODULES_DIR/.env"