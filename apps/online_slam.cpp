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

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <pirvs.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit
 */
void exit_handler(int s) {
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  cv::destroyAllWindows();
  exit(1);
}

/**
 * online_slam tracks the pose of a device while building the map in real-time.
 * The map can then be used for RunTracking() to track the device within the
 * mapped environment. (See online_tracking.) If you wish to build a map with
 * better quality, use offline_slam with a recorded sequence.
 *
 * Prior to running online_slam, it is a good practice to find the best exposure
 * value for the environment where the device will be used. online_features is a
 * good method to find the best exposure value. Set the exposure value in the
 * code.
 */

int main(int argc, char **argv) {
  if (argc < 4) {
    printf("Not enough input argument.\n"
        "Usage:\n%s [calib JSON] [voc JSON] [output sparse map JSON]\n", argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);
  const std::string file_voc(argv[2]);
  const std::string file_map(argv[3]);

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create an initial SLAM state with the ONLINE_SLAM_CONFIG. The
  // ONLINE_SLAM_CONFIG is designed for real-time performance. If a more
  // accurate map is required (and real-time is less than an issue), use
  // OFFLINE_SLAM_CONFIG with a pre-recorded sequence. Note, if using
  // OFFLINE_SLAM_CONFIG for on-line application, the application may end up
  // dropping a lot of frames due to the speed of the SLAM processing, which
  // will result in poor performance.
  std::shared_ptr<PIRVS::SlamState> slam_state;
  if (!PIRVS::InitState(file_calib, PIRVS::ONLINE_SLAM_CONFIG, &slam_state)) {
    printf("Failed to InitState.\n");
    return -1;
  }

  // Create an initial (empty) map. The map will be incrementally built as
  // we feed in new data to RunSlam().
  std::shared_ptr<PIRVS::Map> map;
  if (!PIRVS::InitMap(file_calib, file_voc, &map)) {
    printf("Failed to InitMap.\n");
    return -1;
  }

  // Prepare a drawer to visualize the tracked pose while SLAM runs.
  PIRVS::TrajectoryDrawer drawer;
  cv::Mat img_draw;
  cv::namedWindow("Trajectory");
  // Create an window to show the raw image.
  cv::namedWindow("Left image");

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

  // Please adjust the exposure (from 0 to 2000) based on your environment.
  gDevice->SetExposure(200);

  bool stereo_data_available = false;

  // Stream data from the device and update the SLAM state and the map.
  while (1) {
    // Get the newest data from the device.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }

    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (stereo_data) {
      stereo_data_available = true;
    }

    if (!stereo_data_available) {
      continue;
    }

    // Update the SLAM state and the map according to the data.
    if (!PIRVS::RunSlam(data, map, slam_state)) {
      printf("SLAM failed.\n");
      break;
    }

    // Get the tracking pose from the updated SLAM state and do all sorts of
    // cool stuff with it. Reminder, if the cool stuff takes too long, the
    // device will drop frame which will hurt SLAM (making it more likely to
    // fail). A good practice is to get the pose here, and do the cool stuff at
    // a different thread.
    // Sample code:
    // cv::Affine3d global_T_rig;
    // if (slam_state->GetPose(&global_T_rig)) {
    //  // Cool stuff here.
    // }

    // Visualize the pose after updating the pose with an StereoData.
    if (stereo_data) {
      if (drawer.Draw(slam_state, &img_draw)) {
        cv::imshow("Trajectory", img_draw);
      }
      cv::imshow("Left image", stereo_data->img_l);

      // Press ESC to stop.
      char key = cv::waitKey(1);
      if (key == 27) {
        printf("Stopped.\n");
        break;
      }
    }
  }

  // Save the final map to disk.
  // Note, save the map even if SLAM failed because the map may still be usable.
  printf("Saving map to disk.\n");
  if (!PIRVS::SaveMap(file_map, map)) {
    printf("Failed to save map to disk.\n");
    return -1;
  }

  gDevice->StopDevice();
  cv::destroyAllWindows();

  return 0;
}
