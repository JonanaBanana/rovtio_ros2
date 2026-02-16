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


#include <memory>

#include <Eigen/StdVector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#pragma GCC diagnostic pop

#include "rovtio/RovtioFilter.hpp"
#include "rovtio/RovtioNode.hpp"


#ifdef MAKE_SCENE
#include "rovtio/RovtioScene.hpp"
#endif

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
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
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
      std::cout << "Camera config: " << camera_config << std::endl;
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
  }
}

/**
 * @brief Function to declare the camera config parameters
 * @param ROVTIO node share ptr
 * @return none
 */
void declareParameters(std::shared_ptr<rovtio::RovtioNode<mtFilter>> node)
{
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    node->declare_parameter("camera" + std::to_string(camID)
                            + "_config","");
  }
  node->declare_parameter("filter_config", "");
}


 int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  std::string filter_config;

  // Node
  std::shared_ptr<rovtio::RovtioNode<rovtio::RovtioFilter<rovtio::FilterState<25, 4, 6, 1, 0>>>> node;
  node = std::make_shared<rovtio::RovtioNode<mtFilter>>(mpFilter);
  declareParameters(node);
  node->get_parameter("filter_config", filter_config);
  mpFilter->readFromInfo(filter_config);
  std::cout << "Filter config: " << filter_config << std::endl;
  readCameraConfig(mpFilter, node);
  mpFilter->refreshProperties();
  node->makeTest();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
