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

#include <fstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <pirvs.h>

/**
 * offline_slam is designed to create high quality map from a recorded sequence.
 * The map can then be used for RunTracking() to track the device within the
 * mapped environment. (See online_tracking.)
 *
 * Off-line SLAM requires a pre-recorded sequence. Use online_viewer to record
 * a sequence.
 */

int main(int argc, char **argv) {
  if (argc < 5) {
    printf("Not enough input argument.\n"
           "Usage:\n%s [calib JSON] [voc JSON] [sequence] [output sparse map JSON]\n",
           argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);
  const std::string file_voc(argv[2]);
  const std::string dir_data(argv[3]);
  const std::string file_map(argv[4]);

  // Create an initial SLAM state with the OFFLINE_SLAM_CONFIG. For off-line
  // applications, use OFFLINE_SLAM_CONFIG to produce a more accurate map, and
  // for on-line applications, use ONLINE_SLAM_CONFIG to reach real-time
  // performance.
  std::shared_ptr<PIRVS::SlamState> slam_state;
  if (!PIRVS::InitState(file_calib, PIRVS::OFFLINE_SLAM_CONFIG, &slam_state)){
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

  // Create an data loader to read data from the recorded sequence.
  PIRVS::DataLoader data_loader(dir_data);

  // Go through each (IMU and stereo) data in the sequence to update the SLAM
  // state and the map.
  while (1) {
    // Read the next data in the sequence.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!data_loader.LoadData(&data)) {
      break;
    }

    // Update the SLAM state and the map according to the data.
    if (!PIRVS::RunSlam(data, map, slam_state)) {
      printf("SLAM failed.\n");
      return -1;
    }

    // Get the tracking pose from the updated SLAM state and do all sorts of
    // cool stuff with it.
    cv::Affine3d global_T_rig;
    if (slam_state->GetPose(&global_T_rig)) {
      // Cool stuff here.
    }

    // Visualize the pose after updating the pose with an StereoData.
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (stereo_data) {
      // Use the drawer to visualize the current pose and a short history of
      // trajectory from a top-down view.
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

  cv::destroyAllWindows();

  return 0;
}
