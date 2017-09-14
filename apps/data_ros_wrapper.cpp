/**
 * Copyright 2017 PerceptIn
 *
 * This End-User License Agreement (EULA) is a legal agreement between you
 * (the purchaser) and PerceptIn regarding your use of
 * PerceptIn Robotics Vision System (PIRVS), including PIRVS SDK and
 * associated documentation (the "Software").
 *
 * IF YOU DO NOT AGREE TO ALL OF THE TERMS OF THIS EULA, DO NOT INSTALL,
 * USE OR COPY THE SOFTWARE.
 */

// Don't use  GCC CXX11 ABI by default
#ifndef _GLIBCXX_USE_CXX11_ABI
#define _GLIBCXX_USE_CXX11_ABI 0
#endif

#include <mutex>
#include <thread>
#include <opencv2/highgui.hpp>
#include <pirvs.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit
 */
void exit_handler(int s){
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  cv::destroyAllWindows();
  exit(1);
}

/**
 * online_features visualizes the 2d features + 3d points detected from a device.
 * online_features is also an useful tool for finding the best exposure value
 * for SLAM.
 */

// Callback function for OpenCV trackbar to set exposure of the device.
void ExposureTrackBarCallback(int value, void *ptr) {
  std::shared_ptr<PIRVS::PerceptInDevice>* device_ptr =
      static_cast<std::shared_ptr<PIRVS::PerceptInDevice>*>(ptr);
  if (!device_ptr) {
    return;
  }
  (*device_ptr)->SetExposure(value);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Not enough input argument.\nUsage:\n%s [calib JSON] \n", argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create an initial state for feature detection + matching + triangulation.
  std::shared_ptr<PIRVS::FeatureState> state;
  if (!PIRVS::InitFeatureState(file_calib, &state)){
    printf("Failed to InitFeatureState.\n");
    return -1;
  }

  // Create an interface to stream the PerceptIn V1 device.
  if (!PIRVS::CreatePerceptInV1Device(&gDevice) || !gDevice) {
    printf("Failed to create device.\n");
    return -1;
  }
  // Start streaming from the device.
  if (!gDevice->StartDevice()) {
    printf("Failed to start device.\n");
    return -1;
  }

  // Image and window to draw 2d features.
  cv::Mat img_2d;
  cv::namedWindow("Detected features");
  // Add a trackbar to the window to tune the exposure of the stereo camera.
  uint32_t exposure_value_u;
  if (!gDevice->GetExposure(&exposure_value_u)) {
    printf("Failed to get exposure.\n");
  }
  int exposure_value = exposure_value_u;
  cv::createTrackbar("Exposure", "Detected features", &exposure_value, 2000,
                     ExposureTrackBarCallback, &gDevice);

  // Image and window to draw features with depth.
  cv::Mat img_depth;
  cv::namedWindow("Sparse depth");

  u_int64_t timestamp_last;
  // Stream data from the device and update the feature state.
  while (1) {
    // Get the newest data from the device.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }


    // RunFeature only accept StereoData.
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);

    if (!stereo_data) {
//      continue;
    }
    else{
        std::cout << stereo_data.get() <<std::endl;
        std::cout<< "timestamp of data: "<<data->timestamp <<std::endl;
    }

    std::shared_ptr<const PIRVS::ImuData> imu_data =     std::dynamic_pointer_cast<const PIRVS::ImuData>(data);
    if(!imu_data){
//        continue;
    }
    else if (data->timestamp != timestamp_last){
        std::cout << imu_data.get() <<std::endl;
        std::cout<< "timestamp of data: "<<data->timestamp <<std::endl;
        timestamp_last = data->timestamp;
        std::cout<< "imu acce data: "<<imu_data->accel <<std::endl;
    }

//    std::cout<< "imu acce data: "<<imu_data->accel <<std::endl;
//    std::cout<< "imu gyro data: "<<imu_data->ang_v <<std::endl;


    // Update feature state according to the stereo data.
    // Note, the last input argument of RunFeature() is a flag to specify
    // whether or not to match + triangulate features between the two sensors.
    // Set get_3d = false, if 3d features are not required.
//    const bool get_3d = true;
//    PIRVS::RunFeature(stereo_data, state, get_3d);

//    // Get the 2d features from both sensor in the stereo camera to do all sorts
//    // of cool stuff.
//    //
//    // Sample code:
//    // std::vector<cv::Point2d> features_l;
//    // std::vector<cv::Point2d> feat//    const bool get_3d = true;
//    //    PIRVS::RunFeature(stereo_data, state, get_3d);

//    //    // Get the 2d features from both sensor in the stereo camera to do all sorts
//    //    // of cool stuff.
//    //    //
//    //    // Sample code:
//    //    // std::vector<cv::Point2d> features_l;
//    //    // std::vector<cv::Point2d> features_r;
//    //    // if (state->Get2dFeatures(&features_l, &features_r)) {
//    //    //   // Your cool stuff here.
//    //    // }
//    //    // Get the 3d features from the stereo camera to do all sorts of cool stuff.
//    //    // Note, 3d features are only available if get_3d is true.
//    //    // std::vector<StereoFeature> features;
//    //    // if (get_3d && state->GetStereoFeatures(&features)) {
//    //    //   // Your cool stuff here.
//    //    // }

//    //    // Visualize the 2d detected features on both sensors in the stereo camera.
//    //    if (PIRVS::Draw2dFeatures(stereo_data, state, &img_2d)) {
//    //      cv::imshow("Detected features", img_2d);
//    //    }
//    //    // Visualize the 3d depth of the features.
//    //    if (get_3d && PIRVS::DrawStereoFeatures(stereo_data, state, &img_depth)) {
//    //      cv::imshow("Sparse depth", img_depth);
//    //    }ures_r;
//    // if (state->Get2dFeatures(&features_l, &features_r)) {
//    //   // Your cool stuff here.
//    // }
//    // Get the 3d features from the stereo camera to do all sorts of cool stuff.
//    // Note, 3d features are only available if get_3d is true.
//    // std::vector<StereoFeature> features;
//    // if (get_3d && state->GetStereoFeatures(&features)) {
//    //   // Your cool stuff here.
//    // }

//    // Visualize the 2d detected features on both sensors in the stereo camera.
//    if (PIRVS::Draw2dFeatures(stereo_data, state, &img_2d)) {
//      cv::imshow("Detected features", img_2d);
//    }
//    // Visualize the 3d depth of the features.
//    if (get_3d && PIRVS::DrawStereoFeatures(stereo_data, state, &img_depth)) {
//      cv::imshow("Sparse depth", img_depth);
//    }

    // Press ESC to stop.
    char key = cv::waitKey(1);
    if (key == 27) {
      printf("Stopped.\n");
      break;
    }
  }

  printf("Final exposure value is %d.\n", exposure_value);
  gDevice->StopDevice();
  cv::destroyAllWindows();

  return 0;
}
