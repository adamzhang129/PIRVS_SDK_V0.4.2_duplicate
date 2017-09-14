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
void exit_handler(int s){
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  cv::destroyAllWindows();
  exit(1);
}

/**
 * online_tracking locates a device within a known environment, a pre-built map.
 * Creat the map through offline_slam or online_slam. offline_slam produces a
 * high quality map but takes more time and effort to run, while online_slam is
 * faster (real-time) but the quality of map may not be as good.
 *
 * For best performance, set the exposure value (in the code) to be the same as
 * the value used to build the map (i.e. online_slam or online_viewer when
 * capturing data for offline_slam).
 */

int main(int argc, char **argv) {
  if (argc < 3) {
    printf("Not enough input argument.\n"
           "Usage:\n%s [calib JSON] [input sparse map JSON] \n", argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);
  const std::string file_map(argv[2]);

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create an initial SLAM state with the ONLINE_SLAM_CONFIG, which is designed
  // for on-line applications.
  std::shared_ptr<PIRVS::SlamState> slam_state;
  if (!PIRVS::InitState(file_calib, PIRVS::ONLINE_SLAM_CONFIG, &slam_state)) {
    printf("Failed to InitState.\n");
    return -1;
  }

  // Load the pre-built map from disk.
  std::shared_ptr<PIRVS::Map> map;
  if (!PIRVS::LoadMap(file_map, file_calib, &map)) {
    printf("Failed to LoadMap.\n");
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

  // Stream data from the device and update the SLAM state.
  while (1) {
    // Get the newest data from the device.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }

    // Update SLAM state according to the data.
    PIRVS::RunTracking(data, map, slam_state);

    // Get the tracking pose from the updated SLAM state and do all sorts of
    // cool stuff with it. Reminder, if the cool stuff takes too long, the
    // device will drop frame which will hurt tracking (making it more likely to
    // be lost). A good practice is to get the pose here, and do the cool stuff
    // at a different thread.
    // Sample code:
    // cv::Affine3d global_T_rig;
    // if (slam_state->GetPose(&global_T_rig)) {
    //  // Cool stuff here.
    //}
    // Note, it may take up to 1 sec (~150 data) for RunTracking() to locate
    // the device for the very first time. Thus, the device is usually lost for
    // the first second or so.

    // Visualize the pose after updating the pose with an StereoData.
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
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

  gDevice->StopDevice();
  cv::destroyAllWindows();

  return 0;
}
