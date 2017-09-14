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
  exit(1);
}

/**
 * online_viewer provides a GUI to show the live stream of a device.
 *
 * There is a trackbar on the top window to set the exposure of the stereo
 * camera. Use it to find the best exposure value for the environment where the
 * device will be used.
 *
 * The GUI is also an interface to record from the device. Press space bar to
 * start recording a sequence, and then, press space bar again to stop recording.
 * The recorded sequence will be stored under the specified folder, and the name
 * of the sequence is the timestamp where the sequence begins.
 */

int main(int argc, char **argv) {

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create the interface to stream from a PerceptIn V1 device.
  if (!PIRVS::CreatePerceptInV1Device(&gDevice) || !gDevice) {
    printf("Failed to create device.\n");
    return -1;
  }

  // Start the GUI.
  // The recorded sequence will go to /tmp/PerceptIn_V1_device_recordings/.
  gDevice->GUI("/tmp/PerceptIn_V1_device_recordings/");

  /**
   * Alternatively, you can record data without having a GUI.
   *
   * Example code:
   *   gDevice->StartDevice();
   *   gDevice->StartRecording("/tmp/PerceptIn_V1_device_recordings/");
   *   usleep(3000000);  // Record data for 3 sec.
   *   std::string dir;
   *   size_t num_imu = 0;
   *   size_t num_stereo = 0;
   *   gDevice->StopRecording(&dir, &num_imu, &num_stereo);
   *   printf("Folder: %s\nIMU data: %zd\nStereo data: %zd\n",
   *           dir.c_str(), num_imu, num_stereo);
   *   gDevice->StopDevice();
   */
  return 0;
}
