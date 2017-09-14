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

#ifndef INCLUDE_PIRVS_H
#define INCLUDE_PIRVS_H

#include <list>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

namespace PIRVS {

typedef size_t Timestamp;

/**
 * Define different configurations for SLAM and tracking.
 */
enum SlamConfig {
  /// Prefer speed over accuracy. Designed for online, real-time applications.
  ONLINE_SLAM_CONFIG,
  /// Prefer accuracy over speed. Designed for building an high-quality map
  /// from a recording.
  OFFLINE_SLAM_CONFIG
};

/**
 * A container storing readings from a particular sensor in a PerceptIn
 * device at a particular timestamp.
 */
struct Data {
  Data();
  virtual ~Data();

  /// System timestamp at which the data is captured by the device.
  Timestamp timestamp;
};

/**
 * A container storing readings from the 6-dof IMU from a PerceptIn device.
 */
struct ImuData : public Data {
  /// Acceleration reading from the accelerometer. Unit: meter / sec^2.
  cv::Vec3d accel;
  /// Angular velocity reading from the gyroscope. Unit: radian / sec.
  cv::Vec3d ang_v;
};

/**
 * A container storing readings from the stereo camera from a PerceptIn device.
 */
struct StereoData : public Data {
  /// The image captured from the left sensor of the stereo camera.
  cv::Mat img_l;
  /// The image associated from the left sensor of the stereo camera.
  cv::Mat img_r;
};

/**
 * An interface to load a recorded sequence from the disk.
 */
class DataLoader {
 public:
  /**
   * @brief Constructor.
   *
   * @param dir_data Directory where the data of the sequence are stored in disk.
   */
  DataLoader(const std::string &dir_data);
  ~DataLoader();

  /**
   * @brief Load the next Data in the recorded sequence.
   *
   * @param[out] data Pointer to the shared_ptr to the loaded Data.
   * @return True if a Data is loaded. False (most likely) if the DataLoader
   *         reaches the end of the sequence, or if there are errors loading the
   *         next frame.
   * @note Common errors includes: 1) \p data is NULL; 2) missing one of the
   *       images from the stereo camera; 3) the DataLoader wasn't created
   *       properly due to corrupted sequence.
   */
  bool LoadData(std::shared_ptr<const Data> *data) const;

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

/**
 * A 3d point seen by both sensors in the stereo camera in the PerceptIn device.
 */
struct StereoFeature {
  /// 2d image location of the point in the left sensor of the stereo camera.
  cv::Point2d pt_l;
  /// 2d image location of the point in the right sensor of the stereo camera.
  cv::Point2d pt_r;
  /// 3D point in the left camera's coordinate.
  cv::Point3d pt_3d;
};

/**
 * State of the feature processing system.
 *
 * Use InitFeatureState() to create a FeatureState. Note, InitFeatureState() is
 * the only way to create a FeatureState.
 *
 *     Example:
 *     @code
 *       std::shared_ptr<FeatureState> state;
 *       InitFeatureState("calibration.json", &state);
 *     @endcode
 */
class FeatureState {
 public:
  FeatureState();
  virtual ~FeatureState();

  /**
   * @brief Get the 2d feaatures detected from both sensors in of the stereo
   *        camera.
   * @details The stereo features are available from the FeatureState after it
   *          is updated via RunFeature().
   * @param[out] features_l Pointer to the vector of 2d features detected by the
   *                        left sensor of the stereo.
   * @param[out] features_r Pointer to the vector of 2d features detected by the
   *                        right sensor of the stereo.
   * @return True if the features are available. False if the features are not
   *         available or the pointer \p features is NULL.
   */
  virtual bool Get2dFeatures(std::vector<cv::Point2d> *features_l,
                             std::vector<cv::Point2d> *features_r) const = 0;

  /**
   * @brief Get the stereo features detected from the current StereoData.
   * @details The stereo features are available from the FeatureState if it is
   *          updated via RunFeature() with 3d turned on.
   *
   * @param[out] features Pointer to the vector of stereo features.
   * @return True if the features are available. False if the features are not
   *         available or the pointer \p features is NULL.
   */
  virtual bool GetStereoFeatures(std::vector<StereoFeature> *features) const = 0;
};


/**
 * @brief Create an initial FeatureState.
 *
 * @param file_calib Path to the calibration (.json) file.
 * @param[out] state_ptr Pointer to the shared_ptr to the newly created
 *                       FeatureState.
 * @return True if the state is created (i.e. the shared_ptr is not a nullptr).
 *         False otherwise. If false, the shared_ptr is nullptr.
 * @note Common reasons for returning false includes, 1) the calibration file
 *       does not exist; 2) the calibration file is corrupted; 3) \p state_ptr
 *       is NULL.
 */
bool InitFeatureState(const std::string &file_calib,
                      std::shared_ptr<FeatureState> *state_ptr);

/**
 * @brief Process a StereoData and then update a FeatureState accordingly.
 * @details This function is the main function of the feature processing system,
 *          which detects 2d features from left and right images, matches
 *          features between the two images, and finally, triangulate the
 *          matched features to create 3d points.
 *
 *          Use FeatureState::Get2dFeatures() to query the detected 2d features
 *          and FeatureState::GetStereoFeatures() to query matched features with
 *          their triangulated 3d points.
 *
 * @param stereo_data shared_ptr to the StereoData to process.
 * @param state shared_ptr to the FeatureState to be updated.
 * @param with_3d A flag to specify whether to compute the 3d points or not.
 *                If true the function will match the 2d features between the
 *                left and right images, and then, triangulate to get a 3d point.
 *                If false, this function will only detect 2d features.
 */
void RunFeature(std::shared_ptr<const StereoData> stereo_data,
                std::shared_ptr<FeatureState> state,
                bool with_3d);

/**
 * @brief Draw the detected 2d features for both left and right image.
 * @details This function produces a stereo image with the detected 2d features
 *          in blue hollow circles. The 2d location of these features can be
 *          queried from FeatureState::Get2dFeatures().
 *
 * @param stereo_data shared_ptr to the StereoData where the features are
 *                    detected from.
 * @param state shared_ptr to the FeatureState updated by the \p stereo_data.
 * @param[out] img The output stereo image with the features overlay on top of it.
 *
 * @return True if the \p img is created. False otherwise.
 */
bool Draw2dFeatures(std::shared_ptr<const StereoData> stereo_data,
                    std::shared_ptr<const FeatureState> state,
                    cv::Mat *img);

/**
 * @brief Draw the stereo features on both left and right image.
 * @details This function produces a stereo image with the stereo features
 *          overlay on top of it, color coded according to the depth (z value)
 *          of the 3d points. Blue represents a point with depth equal or less
 *          than 0.08 meter, and red represents depth with 4.0 meter and beyond.
 *          Depth between 0.08 and 4.0 are colored accordingly (i.e. green
 *          represents depth equal to 2.04 meter).
 *
 *          The stereo features can be queried from
 *          FeatureState::GetStereoFeatures().
 *
 * @param stereo_data shared_ptr to the StereoData where the features are
 *                    detected from.
 * @param state shared_ptr to the FeatureState updated by the \p stereo_data.
 * @param[out] img The output stereo image with the features overlay on top of it.
 *
 * @return True if the \p img is created. False otherwise.
 */
bool DrawStereoFeatures(std::shared_ptr<const StereoData> stereo_data,
                        std::shared_ptr<const FeatureState> state,
                        cv::Mat *img);

/**
 * 3D map describing the geometric structure of the observed environment.
 *
 * Use InitMap() to create an empty map, and then, call RunSlam() to incrementally
 * build the Map while localizing the device within that Map. Use SaveMap() to
 * store a Map to the disk. Use LoadMap() to load a pre-built Map from disk, and
 * then call RunTracking() to localize the device within that Map.
 *
 * The coordinate of the Map is defined so that the z-axis is pointing towards
 * the gravity direction.
 */
class Map {
 public:
  Map();
  virtual ~Map();

  /**
   * @brief Get the sparse 3d points in the map.
   *
   * @param[out] points Pointer to the vector of points.
   * @return False if \p points is NULL.
   */
  virtual bool GetPoints(std::vector<cv::Point3d> *points) const = 0;
};

/**
 * @brief Create an initial Map to be used in RunSlam().
 *
 * @param file_calib Path to the calibration (.json) file.
 * @param file_voc Path to the vocabulary (.json) file.
 * @param[out] map_ptr Pointer to the shared_ptr of the newly created Map.
 * @return True if the Map is created. False if failed to create the Map, most
 *         likely because the \p map_ptr is NULL.
 */
bool InitMap(const std::string &file_calib,
             const std::string &file_voc,
             std::shared_ptr<Map> *map_ptr);

/**
 * @brief Load a pre-built Map from disk to be used in RunTracking().
 *
 * @param file_map Path to the (.json) file where the Map is stored.
 * @param file_calib Path to the calibration (.json) file.
 * @param[out] map_ptr Pointer to the shared_ptr of the newly created Map.
 * @return True if the Map is loaded. False if failed.
 * @note Common reasons for returning false includes,p 1) the map file does not
 *       exist or is corrupted; 2) the calibration file does not exist or is
 *       corrupted; 3) \p map_ptr is NULL.
 */
bool LoadMap(const std::string &file_map,
             const std::string &file_calib,
             std::shared_ptr<Map> *map_ptr);

/**
 * @brief Save a Map to disk.
 *
 * @param file_map Path to the (.json) file to write the Map to.
 * @param map Pointer to the shared_ptr of the Map to store.
 * @return True if the Map is saved to disk. False otherwise.
 * @note Common reasons for returning false includes, 1) \p map is nullptr; 2)
 *       \p file_map is empty.
 * @warning This function will overwrite the file if \p file_map already exists.
 */
bool SaveMap(const std::string &file_map,
             std::shared_ptr<const Map> map);

/**
 * The state of the SLAM system at specific timestamp.
 *
 * Use InitState() to create a SlamState.
 */
class SlamState {
 public:
  SlamState();
  virtual ~SlamState();

  /**
   * @brief Get the current pose of the SLAM system.
   * @details The pose is represented by a 4-by-4 transformation that brings a
   *          3d point from the Map coordinate to the coordinate of the device.
   *          The pose is available if the device is "on track" (i.e. the
   *          device is properly localized within the Map).
   *
   *          Example:
   *          @code
   *            // Point in the Map coordinate.
   *            cv::Point2d p_map;
   *            // Same point in the device coordinate.
   *            cv::Point2d p_device = pose * p_map;
   *          @endcode
   *
   * @param[out] pose Pointer to the pose.
   * @return True if pose is available. False otherwise.
   */
  virtual bool GetPose(cv::Affine3d *pose) const = 0;
};

/**
 * @brief Create an initial SlamState to be used in RunSlam() and RunTracking().
 *
 * @param file_calib Path to the calibration (.json) file.
 * @param config The configuration for SLAM or tracking.
 * @param[out] state_ptr Pointer to the shared_ptr of the newly created SlamState.
 * @return True if the SlamState is created. False otherwise.
 * @note Common reasons for returning false includes, 1) the calibration file
 *       does not exist or is corrupted; 2) \p state_ptr is NULL.
 */
bool InitState(const std::string &file_calib,
               const SlamConfig config,
               std::shared_ptr<SlamState> *state_ptr);

/**
 * @brief Process a newly observed Data to update the Map and the SlamState.
 * @details This function is the main function for SLAM (simultaneous
 *          localization/tracking and mapping).
 *
 *          Create the initial SlamState using InitState(), and create the
 *          initial Map using InitMap() to start SLAM.
 *
 *          After each RunSlam(), use SlamState::GetPose() to query the pose of
 *          the device, and use Map::GetPoints() to query the sparse 3d mapped
 *          points.
 *
 * @param data shared_ptr to the Data to process.
 * @param[in,out] map shared_ptr to the Map to be used to localize/track the
 *                    device while being updated.
 * @param[in,out] state shared_ptr to the SlamState to update the pose of the
 *                      device from.
 * @return True if the Map and SlamState are successfully update and the device
 *         is still "on track". False if any of the shared_ptr is nullptr, or if
 *         the SLAM algorithm failed to keep the device on track.
 */
bool RunSlam(std::shared_ptr<const Data> data,
             std::shared_ptr<Map> map,
             std::shared_ptr<SlamState> state);

/**
 * @brief Process a newly observed Data to the SlamState given a known Map.
 * @details This function is the main function for tracking the device within a
 *          known map.
 *
 *          Create the initial SlamState using InitState(), and load a pre-built
 *          Map using LoadMap().
 *
 *          After each RunTracking(), use SlamState::GetPose() to determine
 *          whether the device is still "on track" or not and to query the
 *          current pose of the device (if on track).
 *
 * @param data shared_ptr to the Data to process.
 * @param map shared_ptr to the Map to be used to localize/track the device
 *            while being updated.
 * @param[in,out] state shared_ptr to the SlamState to update the pose of the
 *                      device from.
 */
void RunTracking(std::shared_ptr<const Data> data,
                 std::shared_ptr<const Map> map,
                 std::shared_ptr<SlamState> state);

/**
 * An interface to visualize trajectory of the tracking pose of the device.
 *
 * This interface visualizes the 6-dof pose from the top down view. Thus, the
 * x-axis and the y-axis of the global map are visualized but not the z-axis.
 * Note, the z-axis is pointing towards the gravity direction.
 *
 * The center of the view is the origin of the global coordinate. The x-axis is
 * drawn in blue and the y-axis in red. The length of the axis equals to 1 meter
 * in the physical world. Note, the interface will zoom out as the device moves
 * further away from origin.
 *
 * The current pose of the device is visualized by a circle centered at the
 * (x, y) location of the device and a line pointing out from the circle
 * indicating the heading direction of the device. The trajectory of the past
 * 3 seconds is visualized in a green.
 */
class TrajectoryDrawer {
 public:
  /**
   * @brief Constructor.
   *
   * @param img_size Size of the squared image to visualize the trajectory on.
   */
  TrajectoryDrawer(const size_t img_size = 500);
  ~TrajectoryDrawer();

  /**
   * @brief Update the drawer according to the latest SlamState, and produce an
   *        image with the latest pose.
   *
   * @param state shared_ptr to the latest SlamState.
   * @param[out] img Pointer to the image with the visualization. The size of
   *                 the image is specified in the constructor.
   * @return True if the image is created. False if the \p state is nullptr or
   *         \p img is NULL.
   */
  bool Draw(std::shared_ptr<const SlamState> state,
            cv::Mat *img);

 private:
   class Impl;
   std::shared_ptr<Impl> impl_;
};

/**
 * An interface to stream data from a PerceptIn device.
 *
 * Use CreatePerceptInV1Device() to create an interface for V1 PerceptInDevice.
 */
class PerceptInDevice {
 public:
  PerceptInDevice();
  virtual ~PerceptInDevice();

  /**
   * @brief Start streaming from the device.
   * @return True if device starts streaming. False otherwise.
   * @warning Can not be called if GUI() is on.
   */
  virtual bool StartDevice() = 0;
  /**
   * @brief Stop streaming from the device.
   * @return True if device stops streaming. False otherwise.
   * @warning Can not be called if GUI() is on.
   */
  virtual bool StopDevice() = 0;

  /**
   * @brief Start recording data from the device.
   * @details The caller provides a root folder to store the recorded data, and
   *          StartRecording() will create a folder under the root folder with
   *          the name being the timestamp of the first data in the sequence.
   *
   * @param dir Path to the root folder where the recorded sequence will be
   *            stored.
   * @return True if recording starts. False otherwise.
   * @note Common reasons for returning false includes, 1) device hasn't been
   *       started yet; 2) device is already recording data.
   * @warning Can not be called if GUI() is on.
   */
  virtual bool StartRecording(const std::string &dir = "/tmp/") = 0;

  /**
   * @brief Stop recording data from the device.
   *
   * @param[out] dir Path to the folder where the recorded data sequence is
   *                 stored.
   * @param[out] num_imu Number of IMU data recorded
   * @param[out] num_stereo Number of stereo data recorded.
   * @return True if the device stopped recording. False otherwise.
   * @note Common reasons for returning false includes, 1) device hasn't been
   *       started yet; 2) device is not recording data;
   * @warning Can not be called if GUI() is on.
   */
  virtual bool StopRecording(std::string *dir = nullptr,
                             size_t *num_imu = nullptr,
                             size_t *num_stereo = nullptr) = 0;

  /**
   * @brief Set exposure of the stereo camera.
   *
   * @param value The exposure value to set to.
   * @return True if the exposure value is set to \p value.
   * @warning Can not be called if GUI() is on.
   */
  virtual bool SetExposure(const uint32_t value) = 0;

  /**
   * @brief Read the exposure value of the stereo camera from the device.
   *
   * @param[out] value The exposure value obtained from the device.
   * @return True if the \p value is available. False if failed to query the
   *         exposure value from device.
   * @warning Can not be called if GUI() is on.
   */
  virtual bool GetExposure(uint32_t *value) const = 0;

  /**
   * @brief Get the latest Data from the device.
   * @details This function is intended to be called in a while loop.
   *
   *          Example:
   *          @code
   *            std::shared_ptr<PerceptInDevice> device;
   *            // Create device here.
   *            device->StartDevice();
   *            while (condition) {
   *              std::shared_ptr<const Data> data;
   *              if (!device->GetData(&data)) {
   *                continue;
   *              }
   *              // Process data here.
   *            }
   *          @endcode
   *
   *          Note if the data processing is too slow, this function will drop
   *          frames.
   *
   * @param[out] data_ptr Pointer to the shared_ptr of the newly obtained Data.
   * @return True if data is available. False otherwise.
   * @warning Can not be called if GUI() is on.
   */
  virtual bool GetData(std::shared_ptr<const Data> *data_ptr) = 0;

  /**
   * @brief Start the GUI to view live stream of the stereo camera. The GUI can
   *        also be used to record sequences and to play around with the
   *        exposure values (with a trackbar on the top).
   * @details Once the GUI launched, press space-bar to start recording, and
   *          then press space-bar again to stop recording. Each recording
   *          sequence is stored in a folder under \p dir with the name being
   *          the timestamp of the first data in the sequence.
   *
   * @param dir Path to the root folder where the recorded sequence will be
   *            stored.
   */
  virtual void GUI(const std::string &dir = "/tmp/") = 0;
};

/**
 * @brief Create an interface for V1 PerceptInDevice.
 *
 * @param[out] device_ptr Pointer to the shared_ptr of the newly created device
 *                        interface.
 * @return True if the device is created.
 */
bool CreatePerceptInV1Device(std::shared_ptr<PerceptInDevice> *device_ptr);

}  // namespace PIRVS

#endif  // INCLUDE_PIRVS_H
