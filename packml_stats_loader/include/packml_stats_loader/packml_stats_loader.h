/*
 * Copyright (c) 2019, PlusOne Robotics
 * All rights reserved.
*/

#ifndef SRC_PACKML_STATS_LOADER_H
#define SRC_PACKML_STATS_LOADER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <packml_msgs/GetStats.h>
#include <packml_msgs/LoadStats.h>
#include <packml_msgs/Stats.h>

namespace packml_stats_loader
{

  class PackmlStatsLoader
  {
  public:
    /**
     * @brief Constructor
     * @param pnh Private node handle
     */
    PackmlStatsLoader(const ros::NodeHandle& pnh);
    packml_msgs::Stats loadStats();
    void writeStats(const packml_msgs::GetStats::Response& get_stats_response);

  private:
    ros::NodeHandle pnh_;
    std::string packml_stats_location_;
  };

}

#endif //SRC_PACKML_STATS_LOADER_H
