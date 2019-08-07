/*
 * Copyright (c) 2019, PlusOne Robotics
 * All rights reserved.
*/

#ifndef SRC_PACKML_STATS_LOADER_H
#define SRC_PACKML_STATS_LOADER_H

#include <ros/ros.h>

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

  private:
    ros::NodeHandle pnh_;
  };

}

#endif //SRC_PACKML_STATS_LOADER_H
