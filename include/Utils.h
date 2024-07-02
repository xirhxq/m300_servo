#ifndef FLY_EAGLE_UTILS_H
#define FLY_EAGLE_UTILS_H
#include <math.h>
#include <time.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <queue>

#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#ifndef PROJECT_DJI_SDK_H_H
#define PROJECT_DJI_SDK_H_H
// #include <dji_control.hpp>
// #include <dji_status.hpp>
// #include <dji_version.hpp>
namespace DJISDK {

/*!
 * This enum is used with service query_drone_version to
 * check if the drone is M100 or not. We only support
 * M100 with this particular FW version.
 */
enum DroneFirmwareVersion
{
  M100_31 = 0x03010A00,
};

typedef enum AircraftVersion
{
  UNKNOWN,
  M100,
  M600,
  A3,
  N3,
  M210
} AircraftVersion;

enum FlightControlFlag
{
  HORIZONTAL_ANGLE         = 0x00,
  HORIZONTAL_VELOCITY      = 0x40,
  HORIZONTAL_POSITION      = 0x80,
  // Horizontal angular rate is supported only by A3/N3 based platform
  // and is NOT supported by M100
  HORIZONTAL_ANGULAR_RATE  = 0xC0,

  VERTICAL_VELOCITY = 0x00,
  VERTICAL_POSITION = 0x10,
  VERTICAL_THRUST   = 0x20,

  YAW_ANGLE = 0x00,
  YAW_RATE  = 0x08,

  HORIZONTAL_GROUND = 0x00,
  HORIZONTAL_BODY   = 0x02,

  STABLE_DISABLE = 0x00,
  STABLE_ENABLE  = 0x01
};

/*!
 * Refer to demo_flight_control.cpp in dji_sdk_demo for how to
 * use the display mode.
 */
enum DisplayMode
{
  /*! This mode requires the user to manually
   * control the aircraft to remain stable in air. */
  MODE_MANUAL_CTRL=0,
  /*! In this mode, the aircraft can keep
   * attitude stabilization and only use the
   * barometer for positioning to control the altitude. <br>
   * The aircraft can not autonomously locate and hover stably.*/
  MODE_ATTITUDE=1,

  /*! The aircraft is in normal GPS mode. <br>
   * In normal GPS mode, the aircraft can
   * autonomously locate and hover stably. <br>
   *  The sensitivity of the aircraft to the
   *  command response is moderate.
   */
  MODE_P_GPS=6,
  /*! In hotpoint mode */
  MODE_HOTPOINT_MODE=9,
  /*! In this mode, user can push the throttle
   * stick to complete stable take-off. */
  MODE_ASSISTED_TAKEOFF=10,
  /*! In this mode, the aircraft will autonomously
   * start motor, ascend and finally hover. */
  MODE_AUTO_TAKEOFF=11,
  /*! In this mode, the aircraft can land autonomously. */
  MODE_AUTO_LANDING=12,
  /*! In this mode, the aircraft can antonomously return the
   * last recorded Home Point. <br>
   * There are three types of this mode: Smart RTH(Return-to-Home),
   * Low Batterry RTH, and Failsafe RTTH.  */
  MODE_NAVI_GO_HOME=15,
  /*! In this mode, the aircraft is controled by SDK API. <br>
   * User can directly define the control mode of horizon
   * and vertical directions and send control datas to aircraft. */
  MODE_NAVI_SDK_CTRL=17,
  
  /*! drone is forced to land, might due to low battery */
  MODE_FORCE_AUTO_LANDING=33,
  /*! drone will search for the last position where the rc is not lost */
  MODE_SEARCH_MODE =40,
  /*! Mode for motor starting. <br>
   * Every time user unlock the motor, this will be the first mode. */
  MODE_ENGINE_START = 41
};

/*!
 * Note that the flight status for M100 and A3/N3 are different.
 *
 * Refer to demo_flight_control.cpp in dji_sdk_demo for how to
 * use the flight status.
 *
 */
enum FlightStatus
{
  STATUS_STOPPED   = 0,
  STATUS_ON_GROUND = 1,
  STATUS_IN_AIR    = 2
};

enum M100FlightStatus
{
  M100_STATUS_ON_GROUND        = 1,
  M100_STATUS_TAKINGOFF        = 2,
  M100_STATUS_IN_AIR           = 3,
  M100_STATUS_LANDING          = 4,
  M100_STATUS_FINISHED_LANDING = 5
};

}


#endif //PROJECT_DJI_SDK_H_H


//! ROS standard msgs
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>

//! msgs
#include <dji_osdk_ros/Gimbal.h>
#include <dji_osdk_ros/MobileData.h>
#include <dji_osdk_ros/PayloadData.h>
#include <dji_osdk_ros/FlightAnomaly.h>
#include <dji_osdk_ros/VOPosition.h>
#include <dji_osdk_ros/FCTimeInUTC.h>
#include <dji_osdk_ros/GPSUTC.h>


//! service headers
#include <dji_osdk_ros/Activation.h>
#include <dji_osdk_ros/CameraAction.h>
#include <dji_osdk_ros/DroneArmControl.h>
#include <dji_osdk_ros/DroneTaskControl.h>
#include <dji_osdk_ros/MFIOConfig.h>
#include <dji_osdk_ros/MFIOSetValue.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
#include <dji_osdk_ros/SetLocalPosRef.h>
#include <dji_osdk_ros/SendMobileData.h>
#include <dji_osdk_ros/SendPayloadData.h>
#include <dji_osdk_ros/QueryDroneVersion.h>
#ifdef ADVANCED_SENSING
#include <dji_osdk_ros/Stereo240pSubscription.h>
#include <dji_osdk_ros/StereoDepthSubscription.h>
#include <dji_osdk_ros/StereoVGASubscription.h>
#include <dji_osdk_ros/SetupCameraStream.h>
#endif

// using namespace DJI::OSDK;


#endif
