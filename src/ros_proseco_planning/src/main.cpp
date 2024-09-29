/**
 * @file main.cpp
 * @brief The start of the ProSeCo Planning framework.
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifdef FOUND_CALLGRIND_H
#include "valgrind/callgrind.h"
#endif

#include <ros/console.h>
#include <ros/console_backend.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/rate.h>
#include <iostream>
#include <stdexcept>
#include <string>

#include "ros_proseco_planning/prosecoPlanner.h"

/**
 * @brief 设置ros logger的日志级别为指定级别
 * @details 这通常设置为 fatal，以避免多余的输出
 * @param level 所需的日志级别
 */
void setLogLevel(ros::console::Level level) {
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

int main(int argc, char* argv[]) {
  // 初始化ROS变量
  const std::string packageName = "ros_proseco_planning";
  const auto nodeName           = packageName + "_" + std::string(argv[1]);
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh("~");
  ros::Rate r(1);
  ROS_INFO("11111111111111111111111");
  // Set the log level to error
  ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  //setLogLevel(ros::console::levels::Fatal);
  ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  // Load commandline arguments
  auto packagePath        = ros::package::getPath(packageName);
  std::string optionsArg  = "";
  std::string scenarioArg = "";
  bool absolute_path      = false;
  ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  if (argc > 3) {
    optionsArg  = std::string(argv[2]);
    scenarioArg = std::string(argv[3]);
    if (argc > 4) {
      absolute_path = std::string(argv[4]) == "true" ? true : false;
    }
  } else {
    ROS_ERROR_STREAM("TOO FEW ARGUMENTS! NEED OPTIONS AND SCENARIO");
    return -1;
  }
  ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
  // Create the ProSeCo planner
  auto prosecoPlanner =
      proseco_planning::ProSeCoPlanner(packagePath, optionsArg, scenarioArg, absolute_path);

  // START OF ROS LOOP
  while (ros::ok()) {
// Start the callgrind recording
#ifdef FOUND_CALLGRIND_H
    CALLGRIND_START_INSTRUMENTATION;
#endif

    try {
      ROS_INFO("11111111111111111111111");
      prosecoPlanner.plan();
      prosecoPlanner.save();
    } catch (const std::runtime_error& e) {
      std::cout << "ProSeCo planner could not generate valid actions." << std::endl;
      return -1;
    }

// Stop the callgrind recording
#ifdef FOUND_CALLGRIND_H
    CALLGRIND_STOP_INSTRUMENTATION;
#endif
    // Export a summary of the results
    //prosecoPlanner.save();

    // Exit ROS loop
    break;

  }  // END OF ROS LOOP

  return 0;
}