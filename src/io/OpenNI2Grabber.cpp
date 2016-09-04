/*
 * Copyright (c) 2016, Jihoon Lee
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */


#include <ecto_pcl/ecto_pcl.hpp>

#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <boost/chrono.hpp>

#include "pcl/io/openni2/openni.h"

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef pcl::io::OpenNI2Grabber O2Grabber;
typedef pcl::io::openni2::Image PCLImage;
typedef boost::shared_ptr<pcl::io::openni2::Image> PCLImageConstPtr;

class SimpleOpenNI2Grabber
{
public:
  SimpleOpenNI2Grabber() : thread_(boost::ref(*this))

  {
    runmutex_.lock(); // when this gets unlocked in the destructor, the thread unblocks
  }

  ~SimpleOpenNI2Grabber() {}

  void stop() {
    runmutex_.unlock();
    thread_.join();
  }

  void operator()()
  {
    boost::scoped_ptr<O2Grabber> interface(new O2Grabber);
    boost::signals2::connection c_cloud;
    //boost::signals2::connection c_rgb;
    //boost::signals2::connection c_depth;

    boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&SimpleOpenNI2Grabber::cloud_callback, this, _1);
    //boost::function<void (const PCLImageConstPtr&) > rgb_image_cb   = boost::bind (&SimpleOpenNI2Grabber::rgb_image_callback, this, _1);
    //boost::function<void (const PCLImageConstPtr&) > depth_image_cb = boost::bind (&SimpleOpenNI2Grabber::depth_image_callback, this, _1);
    c_cloud = interface->registerCallback(cloud_cb);
    //c_rgb= interface->registerCallback(rgb_image_cb);
    //c_depth = interface->registerCallback(depth_image_cb);

    interface->start();
    runmutex_.lock(); // blocks until the destructor unlocks it
    runmutex_.unlock(); // unlock it again... why does this stop a crash?

    c_cloud.disconnect();
    //c_rgb.disconnect();
    //c_depth.disconnect();

    interface->stop();
  }

  CloudPOINTXYZRGBA::ConstPtr getLatestXYZRGBCloud()
  {
    boost::mutex::scoped_lock lock(datamutex_cloud_);

    while (!cloud_)
      cond_.wait(lock);
    CloudPOINTXYZRGBA::ConstPtr p = cloud_;
    cloud_.reset();
    return p;
  }

  /*
  PCLImageConstPtr getLatestRGBImage()
  {
    boost::mutex::scoped_lock lock(dm_rgb_);

    while (!cloud_)
      cond_.wait(lock);
    PCLImageConstPtr p = rgb_;
    rgb_.reset();
    return p;
  }

  PCLImageConstPtr getLatestDepthImage()
  {
    boost::mutex::scoped_lock lock(dm_depth_);

    while (!cloud_)
      cond_.wait(lock);
    PCLImageConstPtr p = depth_;
    depth_.reset();
    return p;
  }
  */

  void
  cloud_callback (const CloudConstPtr& cloud)
  {
    boost::mutex::scoped_lock lock(datamutex_cloud_);
    cloud_ = cloud;
    cond_.notify_one();
  }

  /*
  void rgb_image_callback(const PCLImageConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(dm_rgb_);
    rgb_ = msg;
    cond_.notify_one();
  }

  void depth_image_callback(const PCLImageConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(dm_depth_);
    depth_ = msg;
    cond_.notify_one();
  }
  */

  boost::mutex datamutex_cloud_, dm_rgb_, dm_depth_;
  boost::mutex runmutex_;
  boost::condition_variable cond_;

  CloudConstPtr cloud_;
  PCLImageConstPtr rgb_, depth_;
  boost::thread thread_;
};

namespace ecto {
  namespace pcl {
    struct OpenNI2Grabber
    {
      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare<PointCloud> ("pointcloud", "An XYZ/XYZRGB point cloud from the openni2");
        //    outputs.declare<PCLImage> ("rgb", "rgb image");
        //    outputs.declare<PCLImage> ("depth", "depth image");
      }

      void configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
      {
        impl_.reset(new SimpleOpenNI2Grabber());
        out_pointcloud_ = outputs["pointcloud"];
        // out_rgb_ = outputs["rgb"];
        // out_depth_ = outputs["depth"];
      }

      int process(const tendrils& /*inputs*/, const tendrils& outputs)
      {
        PointCloud p(impl_->getLatestXYZRGBCloud());
        *out_pointcloud_ = p;
        /*
        PCLImage rgb(impl_->getLatestRGBImage());
        *out_rgb_ = rgb;
        PCLImage depth(impl_->getLatestDepthImage());
        *out_depth_ = depth;
        */
        return ecto::OK;
      }

      ~OpenNI2Grabber() {
        if(impl_)
          impl_->stop();
      }
      boost::scoped_ptr<SimpleOpenNI2Grabber> impl_;
      spore<PointCloud> out_pointcloud_;
      // spore<PCLImage> out_rgb_, out_depth_;
    };
  }
}

//ECTO_CELL(ecto_pcl, ecto::pcl::SimpleOpenNI2Grabber, "SimpleOpenNI2Grabber", "Simple OpenNI2 grabber");
ECTO_CELL(ecto_pcl, ecto::pcl::OpenNI2Grabber, "OpenNI2Grabber", "Grabber from openni2 device");
