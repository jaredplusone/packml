/*
 * Copyright (c) 2019, PlusOne Robotics
 * All rights reserved.
*/

#include "packml_stats_loader/packml_stats_loader.h"

namespace packml_stats_loader
{
  PackmlStatsLoader::PackmlStatsLoader(const ros::NodeHandle &pnh): pnh_(pnh)
  {

    // Get stats location
    if (!pnh_.getParam("packml_stats_location", packml_stats_location_))
    {
      ROS_FATAL_STREAM("Missing param: packml_stats_location. Unable to load or save stats. Shutting down.");
      ros::shutdown();
    }

    // Load stats
    if (!pnh_.getParam("load_stats", load_stats_))
    {
      ROS_WARN_STREAM("Missing param: load_stats. Defaulting to false.");
      load_stats_ = false;
    }
    if (load_stats_)
    {
      if (!loadStats())
      {
        ROS_ERROR_STREAM("Failed to load stats");
      }
      else
      {
        ROS_INFO_STREAM("Successfully loaded stats");
      }

    }

    // Save stats
    ros::ServiceClient get_stats_client = pnh_.serviceClient<packml_msgs::GetStats>("get_stats");
    ros::service::waitForService(get_stats_client.getService());
    packml_msgs::GetStats get_stats_srv;
    if (!get_stats_client.call(get_stats_srv))
    {
      ROS_ERROR_STREAM("Failed to call service " << get_stats_client.getService());
    }
    else
    {
      if (!writeStats(get_stats_srv.response))
      {
        ROS_ERROR_STREAM("Failed to write stats");
      }
      else
      {
        ROS_INFO_STREAM("Successfully saved stats");
      }
    }

  }

  bool PackmlStatsLoader::loadStats()
  {
    rosbag::Bag bag;
    try
    {
      bag.open(packml_stats_location_, rosbag::bagmode::Read);
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM(ex.what());
      return false;
    }

    for(const auto& message_instance : rosbag::View(bag))
    {
      if (message_instance.isType<packml_msgs::Stats>())
      {
        auto message = message_instance.instantiate<packml_msgs::Stats>();
        ROS_INFO_STREAM(message->availability);
      }
    }

    bag.close();
    return true;
  }

  bool PackmlStatsLoader::writeStats(const packml_msgs::GetStats::Response& get_stats_response)
  {
    packml_msgs::Stats stats_msg = get_stats_response.stats;
    rosbag::Bag bag;

    try
    {
      bag.open(packml_stats_location_, rosbag::bagmode::Write);
    }
    catch (const std::exception &ex)
    {
      ROS_ERROR_STREAM(ex.what());
      return false;
    }

    bag.write("stats", ros::Time::now(), stats_msg);

    bag.close();
    return true;
  }

}