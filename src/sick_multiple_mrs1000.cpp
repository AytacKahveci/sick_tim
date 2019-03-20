/*
 * Copyright (C) 2017, Osnabrück University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author:
 *         Sebastian Pütz <spuetz@uos.de>
 *
 */

#include <sick_tim/sick_mrs1000_communication.h>
#include <sick_tim/sick_mrs1000_parser.h>
#include <vector>
#include <boost/thread.hpp>
#include <stdexcept>
#include <memory>

boost::mutex read_mutex;
boost::condition_variable update_cond;

int readThread(sick_tim::SickTimCommon* s, const std::string& hostname, sick_tim::SickTimConfig conf)
{
  try
  {
    boost::this_thread::interruption_point();
    int result = sick_tim::ExitError;

    result = s->loopOnce(conf);

    if (result == sick_tim::ExitFatal)
    { 
      ROS_INFO_STREAM("Thread id:" << boost::this_thread::get_id() << "is returned with ExitFatal flag. Hostname: " << hostname);
      return result;
    }
  }
  catch(boost::thread_interrupted&)
  {
    return sick_tim::ExitSuccess;
  }  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_mrs_1000");
  ros::NodeHandle nhPriv("~");

  // check for TCP - use if ~hostname is set.
  bool useTCP = false;
  std::vector<std::string> hostname;
  if(!nhPriv.getParam("hostname", hostname))
  {
    ROS_FATAL_STREAM("No hostname given!");
    return sick_tim::ExitError;
  }
  
  int num_device = hostname.size();
  ROS_INFO("%d numbers of device used.", num_device);

  std::string port;
  nhPriv.param<std::string>("port", port, "2112");

  std::vector<sick_tim::SickMRS1000Parser*> parser;
  for(int i = 0; i < num_device; ++i)
  { 
    parser[i] = new sick_tim::SickMRS1000Parser();
  }

  std::vector<double> param;
  if (nhPriv.getParam("range_min", param))
  {
    if(param.size() != num_device)
    {
      printf("Default params used for range_min");
      param.resize(num_device);
      param.assign(num_device, 0.2);
    }
    for(int i = 0; i < num_device; ++i)
      parser[i]->set_range_min(param[i]);
  }
  if (nhPriv.getParam("range_max", param))
  {
    if(param.size() != num_device)
    {
      printf("Default params used for range_max");
      param.resize(num_device);
      param.assign(num_device, 64);
    }
    for(int i = 0; i < num_device; ++i)
      parser[i]->set_range_max(param[i]);
  }
  if (nhPriv.getParam("time_increment", param))
  {
    if(param.size() != num_device)
    {
      printf("Default params used for range_max");
      param.resize(num_device);
      param.assign(num_device, 0.0001);
    }
    for(int i = 0; i < num_device; ++i)
      parser[i]->set_time_increment(param[i]);
  }

  int timelimit;
  nhPriv.param("timelimit", timelimit, 5);


  std::vector<sick_tim::SickTimConfig> configurations(2);

  configurations[0].frame_id = "frontlidar_top";
  configurations[0].min_ang = -2.35619449019;
  configurations[0].max_ang = 2.35619449019;
  configurations[0].time_offset = 0.00001349;
  
  configurations[1].frame_id = "rearlidar_top";
  configurations[1].min_ang = -2.35619449019;
  configurations[1].max_ang = 2.35619449019;
  configurations[1].time_offset = 0.00001349;


  std::vector<sick_tim::SickTimCommon*> s(num_device, NULL);

  int result = sick_tim::ExitError;

  std::vector<std::string> scan_topic_names = {"/front/scan", "/rear/scan"};
  std::vector<std::string> cloud_topic_names = {"/front/cloud", "/rear/cloud"};

  for(int i = 0; i < num_device; ++i)
  {
    s[i] = new sick_tim::SickMrs1000Communication(hostname[i], port, timelimit, parser[i]);

    result = s[i]->init();

    if(result != sick_tim::ExitSuccess)
    {
      std::stringstream ss;
      ss << "Error while initializing device with IP: " << hostname[i] << '\n';
      throw std::runtime_error(ss.str());
    }
  }

  while (ros::ok())
  {
    std::vector<boost::thread*> th;
    for(int i = 0; i < num_device; ++i)
    {
      th.push_back(new boost::thread(boost::bind(&readThread, s[i], hostname[i], configurations[i])) );
    }
    for(int i = 0; i < num_device; ++i)
    {
      th[i]->join();
      delete th[i];

      if (result == sick_tim::ExitFatal)
        return result; // TODO close all threads
    }
  }

  return result;
}