/*
 * Copyright (c) 2019, PlusOne Robotics
 * All rights reserved.
*/

#include "packml_stats_loader/packml_stats_loader.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "packml_stats_loader");
  ros::NodeHandle pnh("~");

  packml_stats_loader::PackmlStatsLoader packml_stats_loader(pnh);
}
