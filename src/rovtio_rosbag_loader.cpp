/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include "rovtio/RovtioFilter.hpp"
#include "rovtio/RovtioNode.hpp"
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "rovtio/RovtioNode.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#define foreach BOOST_FOREACH

#ifdef ROVTIO_NMAXFEATURE
static constexpr int nMax_ = ROVTIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVTIO_NLEVELS
static constexpr int nLevels_ = ROVTIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVTIO_PATCHSIZE
static constexpr int patchSize_ = ROVTIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVTIO_NCAM
static constexpr int nCam_ = ROVTIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVTIO_NPOSE
static constexpr int nPose_ = ROVTIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovtio::RovtioFilter<rovtio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

/**
 * @brief function to deserialize a template sample message, to read from a rosbag.
 * @tparam T
 * @param msg
 * @return T type. Msg type/
 */
template <typename T>
T deserializeMessage(auto msg) {
  rclcpp::SerializedMessage extractedMsg(*msg->serialized_data);
  rclcpp::Serialization<T> deserializer;
  T returnMsg;
  deserializer.deserialize_message(&extractedMsg, &returnMsg);
  return returnMsg;
}


/**
 *
 * @brief function to serialize the message, to store in the bag file.
 * @tparam Message type
 * @param msg to serialize
 * @return generalized serialized msg to store in bag file
 */
template <typename T>
rclcpp::SerializedMessage serializeMessage(T msgToSerialize ) {
  rclcpp::SerializedMessage serializedMsg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msgToSerialize, &serializedMsg);
  return serializedMsg;
}


/**
 * @brief Function to convert the Point stamped message to the pose stamped message
 * @param PointStamped message reference
 * @param PoseStamped message reference
 * @return None
 */
void pointToPose(const geometry_msgs::msg::PointStamped pointMsg,
                geometry_msgs::msg::PoseStamped::SharedPtr poseMsg) {
  poseMsg->header.stamp = pointMsg.header.stamp;
  poseMsg->header.frame_id = pointMsg.header.frame_id;
  poseMsg->pose.position.x = pointMsg.point.x;
  poseMsg->pose.position.y = pointMsg.point.y;
  poseMsg->pose.position.z = pointMsg.point.z;
  poseMsg->pose.orientation.x = 0;
  poseMsg->pose.orientation.y = 0;
  poseMsg->pose.orientation.z = 0;
  poseMsg->pose.orientation.w = 1;
}


/**
 * @brief Function to declare the camera config parameters
 * @param ROVTIO node share ptr
 * @return none
 */
void declareParameters(std::shared_ptr<rovtio::RovtioNode<mtFilter>> node) {
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    node->declare_parameter("camera" + std::to_string(camID)
                            + "_config","");
  }
  node->declare_parameter("filter_config", "");
  node->declare_parameter("record_odometry", true);
  node->declare_parameter("record_pose_with_covariance_stamped", true);
  node->declare_parameter("record_transform", true);
  node->declare_parameter("record_extrinsics", true);
  node->declare_parameter("record_imu_bias", true);
  node->declare_parameter("record_pcl", true);
  node->declare_parameter("record_markers", true);
  node->declare_parameter("record_patch", false);
  node->declare_parameter("reset_trigger", 0.0);
}


/**
 * @brief Function to read the camera calibration parameters. File path are ros params of node.
 * @param mpFilter
 * @param node
 * @return None
 */
void readCameraConfig(std::shared_ptr<mtFilter> mpFilter,
                      std::shared_ptr<rovtio::RovtioNode<mtFilter>> node) {
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    if (node->get_parameter("camera" + std::to_string(camID)
                            + "_config", camera_config)) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
                            }
  }
}


int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  std::string filter_config;

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  auto rovtioNode = std::make_shared<rovtio::RovtioNode<mtFilter>>(mpFilter);
  declareParameters(rovtioNode);
  rovtioNode->get_parameter("filter_config", filter_config);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  readCameraConfig(mpFilter, rovtioNode);
  mpFilter->refreshProperties();
  
  rovtioNode->makeTest();
  double resetTrigger = 0.0;
  rovtioNode->get_parameter("record_odometry", rovtioNode->forceOdometryPublishing_);
  rovtioNode->get_parameter("record_pose_with_covariance_stamped", rovtioNode->forcePoseWithCovariancePublishing_);
  rovtioNode->get_parameter("record_transform", rovtioNode->forceTransformPublishing_);
  rovtioNode->get_parameter("record_extrinsics", rovtioNode->forceExtrinsicsPublishing_);
  rovtioNode->get_parameter("record_imu_bias", rovtioNode->forceImuBiasPublishing_);
  rovtioNode->get_parameter("record_pcl", rovtioNode->forcePclPublishing_);
  rovtioNode->get_parameter("record_markers", rovtioNode->forceMarkersPublishing_);
  rovtioNode->get_parameter("record_patch", rovtioNode->forcePatchPublishing_);
  rovtioNode->get_parameter("reset_trigger", resetTrigger);

  std::cout << "Recording";
  if(rovtioNode->forceOdometryPublishing_) std::cout << ", odometry";
  if(rovtioNode->forceTransformPublishing_) std::cout << ", transform";
  if(rovtioNode->forceExtrinsicsPublishing_) std::cout << ", extrinsics";
  if(rovtioNode->forceImuBiasPublishing_) std::cout << ", imu biases";
  if(rovtioNode->forcePclPublishing_) std::cout << ", point cloud";
  if(rovtioNode->forceMarkersPublishing_) std::cout << ", markers";
  if(rovtioNode->forcePatchPublishing_) std::cout << ", patch data";
  std::cout << std::endl;

  rosbag2_cpp::Reader bagIn;
  std::string rosbag_filename = "dataset.bag";
  rovtioNode->declare_parameter("rosbag_filename", rosbag_filename);
  rovtioNode->get_parameter("rosbag_filename", rosbag_filename);
  bagIn.open(rosbag_filename);

  rosbag2_cpp::Writer bagOut;
  std::size_t found = rosbag_filename.find_last_of("/");
  std::string file_path = rosbag_filename.substr(0,found);
  std::string file_name = rosbag_filename.substr(found+1);
  if(file_path==rosbag_filename){
    file_path = ".";
    file_name = rosbag_filename;
  }

  std::stringstream stream;
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format("%Y-%m-%d-%H-%M-%S");
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << rovtioNode->get_clock()->now().seconds() << "_" << nMax_ << "_" << nLevels_ << "_" << patchSize_ << "_" << nCam_  << "_" << nPose_;
  std::string filename_out = file_path + "/rovtio/" + stream.str();
  rovtioNode->declare_parameter("filename_out", filename_out);
  rovtioNode->get_parameter("filename_out", filename_out);
  std::string rosbag_filename_out = filename_out + ".bag";
  std::string info_filename_out = filename_out + ".info";
  std::string gt_topic_name = "/gt";
  rovtioNode->declare_parameter("gt_topic_name", gt_topic_name);
  rovtioNode->get_parameter("gt_topic_name",gt_topic_name);
  std::cout << "GT topic: " << gt_topic_name << std::endl;
  bagOut.open(rosbag_filename_out);

  // Copy info
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();
  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  rovtioNode->declare_parameter("imu_topic_name", imu_topic_name);
  rovtioNode->get_parameter("imu_topic_name", imu_topic_name);
  std::string cam0_topic_name = "/cam0/image_raw";
  rovtioNode->declare_parameter("cam0_topic_name", cam0_topic_name);
  rovtioNode->get_parameter("cam0_topic_name", cam0_topic_name);
  std::string cam1_topic_name = "/cam1/image_raw";
  rovtioNode->declare_parameter("cam1_topic_name", cam1_topic_name);
  rovtioNode->get_parameter("cam1_topic_name", cam1_topic_name);
  std::string odometry_topic_name = rovtioNode->pubOdometry_->get_topic_name();
  std::string transform_topic_name = rovtioNode->pubTransform_->get_topic_name();
  std::string extrinsics_topic_name[mtFilter::mtState::nCam_];
  for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
    extrinsics_topic_name[camID] = rovtioNode->pubExtrinsics_[camID]->get_topic_name();
  }
  std::string imu_bias_topic_name = rovtioNode->pubImuBias_->get_topic_name();
  std::string pcl_topic_name = rovtioNode->pubPcl_->get_topic_name();
  std::string u_rays_topic_name = rovtioNode->pubMarkers_->get_topic_name();
  std::string patch_topic_name = rovtioNode->pubPatch_->get_topic_name();

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  topics.push_back(std::string(cam1_topic_name));


  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;
  while (bagIn.has_next()) {
    auto serializedMsg = bagIn.read_next();
    if(serializedMsg->topic_name == imu_topic_name){
      sensor_msgs::msg::Imu imuMsg = deserializeMessage<sensor_msgs::msg::Imu>(serializedMsg);
      sensor_msgs::msg::Imu::ConstPtr imuMsgPtr = std::make_shared<sensor_msgs::msg::Imu>(imuMsg);
      if (imuMsgPtr != NULL) rovtioNode->imuCallback(std::const_pointer_cast<sensor_msgs::msg::Imu>(imuMsgPtr));
    }
    if(serializedMsg->topic_name == cam0_topic_name){
      sensor_msgs::msg::Image imgMsg =  deserializeMessage<sensor_msgs::msg::Image>(serializedMsg);
      sensor_msgs::msg::Image::ConstPtr imgMsgPtr = std::make_shared<sensor_msgs::msg::Image>(imgMsg);
      if (imgMsgPtr != NULL) rovtioNode->imgCallback(std::const_pointer_cast<sensor_msgs::msg::Image>(imgMsgPtr), 0);
    }
    if(serializedMsg->topic_name == cam1_topic_name){
      sensor_msgs::msg::Image imgMsg2 = deserializeMessage<sensor_msgs::msg::Image>(serializedMsg);
      sensor_msgs::msg::Image::ConstPtr imgMsg2Ptr = std::make_shared<sensor_msgs::msg::Image>(imgMsg2);
      if (imgMsg2Ptr != NULL) rovtioNode->imgCallback(std::const_pointer_cast<sensor_msgs::msg::Image>(imgMsg2Ptr), 1);
    }
	if(serializedMsg->topic_name == gt_topic_name) {
		geometry_msgs::msg::PointStamped gtPose = deserializeMessage<geometry_msgs::msg::PointStamped>(serializedMsg);
    geometry_msgs::msg::PoseStamped::SharedPtr poseMsgPtr = std::make_shared<geometry_msgs::msg::PoseStamped>();
	  pointToPose(gtPose, poseMsgPtr);
	  bagOut.write(*poseMsgPtr, gt_topic_name, rovtioNode->get_clock()->now());
	}
    rclcpp::spin_some(rovtioNode);

    if(rovtioNode->gotFirstMessages_){
      static double lastSafeTime = rovtioNode->mpFilter_->safe_.t_;
      if(rovtioNode->mpFilter_->safe_.t_ > lastSafeTime){
        if(rovtioNode->forceOdometryPublishing_)
        {
          bagOut.write(rovtioNode->odometryMsg_, odometry_topic_name, rovtioNode->get_clock()->now());
        }
        //if(rovtioNode->forceTransformPublishing_) bagOut.write(transform_topic_name,rovtioNode->get_clock()->now(),rovtioNode->transformMsg_);
        for(int camID=0;camID<mtFilter::mtState::nCam_;camID++)
        {
          if(rovtioNode->forceExtrinsicsPublishing_) {
            bagOut.write(rovtioNode->extrinsicsMsg_[camID],
              extrinsics_topic_name[camID],rovtioNode->get_clock()->now());
          }
        }
        if(rovtioNode->forceImuBiasPublishing_) {
          bagOut.write(rovtioNode->imuBiasMsg_,imu_bias_topic_name,rovtioNode->get_clock()->now());
        }
        if(rovtioNode->forcePclPublishing_) {
          bagOut.write(rovtioNode->pclMsg_, pcl_topic_name, rovtioNode->get_clock()->now());
        }
        if(rovtioNode->forceMarkersPublishing_) {
          bagOut.write(rovtioNode->odometryMsg_,odometry_topic_name,rovtioNode->get_clock()->now());
        }
          if(rovtioNode->forcePatchPublishing_) {
            bagOut.write(rovtioNode->patchMsg_, patch_topic_name, rovtioNode->get_clock()->now());
          }
        lastSafeTime = rovtioNode->mpFilter_->safe_.t_;
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        rovtioNode->requestReset();
        rovtioNode->mpFilter_->init_.state_.WrWM() = rovtioNode->mpFilter_->safe_.state_.WrWM();
        rovtioNode->mpFilter_->init_.state_.qWM() = rovtioNode->mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
  }

  bagOut.close();
  bagIn.close();


  return 0;
}
