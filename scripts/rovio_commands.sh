#!/bin/bash

#Author: Suyash Yeotikar
#This file contains helper functions to execute commands of the rovio package.
#Usage: source rovio_commands.sh . Then execute any of the functions below.
#Note: run all commands in this file from rovio_ws folder in a terminal

#function to convert euroc datasets from ros 1 to ros2 bag formats
function convert_euroc_rosbags(){
  DATASETS_DIR=$(pwd)/datasets/machine_hall
  folders=($( ls $DATASETS_DIR))
  for fol in "${folders[@]}"; do
    ROS2_DIR=$DATASETS_DIR/$fol/${fol}_ros2
    if [[ -d $ROS2_DIR ]]; then
      echo "ROS2 dir exists. No need to convert dataset: ${ROS2_DIR}"
    else
      echo "Converting dataset: ${fol}"
      rosbags-convert --dst ${ROS2_DIR} $DATASETS_DIR/${fol}/${fol}.bag
    fi

  done

}

#function to install euroc datasets
function install_euroc_datasets() {
  DATASETS_DIR=$(pwd)/datasets
  EUROC_LINK="https://www.research-collection.ethz.ch/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/download"
  mkdir -p $DATASETS_DIR
  wget -P $DATASETS_DIR $EUROC_LINK
  unzip -d ${DATASETS_DIR} ${DATASETS_DIR}/download
  rm -rf ${DATASETS_DIR}/download
}

#function to wait for ROVIO to finish completion
function wait_for_rovio() {
  echo "Waiting for ROVIO to start..."
  until ros2 node list | grep -q "/rovio" > /dev/null; do
    sleep 1
  done

  echo "ROVIO running, waiting for completion..."
  while ros2 node list | grep -qx "/rovio" > /dev/null; do
    sleep 1
  done

  echo "ROVIO finished"
}


#function to run rovio on euroc datasets
function run_rovio_euroc() {
  EUROC_DATASETS_LOCATION=~/datasets
  ROVIO_WS=~/rovtio_ros2_ws/install/setup.bash
  dirList=($(ls ${EUROC_DATASETS_LOCATION}))
  source $ROVIO_WS
  for dir in "${dirList[@]}"; do
    ROS2_BAG_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/${dir}.db3
    ROVIO_OUTPUT_LOCATION=${EUROC_DATASETS_LOCATION}/${dir}/rovio/
    echo "Deleting prevous rovio output location"
    rm -rf ${ROVIO_OUTPUT_LOCATION}
    echo "Processing dataset: ${ROS2_BAG_LOCATION}"
    if [ -f ${ROS2_BAG_LOCATION} ]; then
      ros2 launch rovio ros2_rovio_rosbag_loader_launch.yaml rosbag_filename:=$ROS2_BAG_LOCATION &
      wait_for_rovio
      killall -9 image_view
    else
      echo "Skipping dataset: ${dir}, no ros2 bag file found"
    fi
  done
}


#function to run rovio on euroc datasets using the live verson
run_rovio_euroc_live() {
  EUROC_DATASETS_LOCATION=$(pwd)/datasets/machine_hall
  ROVIO_WS=$(pwd)/install/setup.bash
  dirList=($(ls ${EUROC_DATASETS_LOCATION}))
  source $ROVIO_WS

  for dir in "${dirList[@]}"; do
    ROS2_BAG_LOCATION="${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/${dir}_ros2.db3"
    ROVIO_OUTPUT_LOCATION="${EUROC_DATASETS_LOCATION}/${dir}/${dir}_ros2/rovio_live"
    mkdir -p "${ROVIO_OUTPUT_LOCATION}"

    echo "Processing dataset: ${ROS2_BAG_LOCATION}"
    if [ -f "${ROS2_BAG_LOCATION}" ]; then
      # Launch ROVIO in background
      ros2 launch rovio ros2_rovio_node_launch.yaml &
      ROVIO_PID=$!

      # Wait until /clock is available
      until ros2 topic list | grep -q "/clock"; do
        sleep 0.2
      done

      # Start recording in background
      nohup ros2 bag record -a -o "${ROVIO_OUTPUT_LOCATION}/rovio_live_output.bag" > /dev/null 2>&1 &
      BAG_RECORD_PID=$!

      # Play bag in foreground → script will wait until finished
      ros2 bag play "${ROS2_BAG_LOCATION}" --clock

      # After playback finishes, stop recording and ROVIO
      killall -9 rovio_node
      killall -9 image_view
      killall -9 ros2

      echo "Finished dataset: ${dir}"
    else
      echo "Skipping dataset: ${dir}, no ros2 bag file found"
    fi
  done
}

#function to evaluate rovio trajectory on euroc dataset results. Generates plots and ATE RPE metrics
function evaluate_rovio_euroc() {
  DATASETS_DIR="$(pwd)/datasets/machine_hall"
  DIR_LIST=($(ls $DATASETS_DIR ))
  GT_TOPIC="/leica/position"
  ODOM_TOPIC="/rovio/odometry"
  for dir in "${DIR_LIST[@]}"; do
    ROVIO_RESULT=${DATASETS_DIR}/$dir/${dir}_ros2/rovio/
    BAG_FILE=$(ls $ROVIO_RESULT | grep "bag")
    echo "Evaluating bag file: ${BAG_FILE}"
    BAG_LOCATION=${ROVIO_RESULT}/${BAG_FILE}
    evo_ape bag2 ${BAG_LOCATION} ${GT_TOPIC} ${ODOM_TOPIC} -va --save_results ${ROVIO_RESULT}/${dir}_ape_results.zip
    evo_res ${ROVIO_RESULT}/${dir}_ape_results.zip --save_table ${ROVIO_RESULT}/${dir}_ape_rovio.csv
  done
}



