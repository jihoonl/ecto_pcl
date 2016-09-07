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

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cfloat>
#include <pcl/visualization/eigen.h>
//#include <pcl/visualization/vtk.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>6)
#include <pcl/visualization/pcl_plotter.h>
#endif
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>
#include <vtkPolyDataReader.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef ::pcl::visualization::PCLVisualizer PCLVisualizer;

class Visualizer
{
public:
  Visualizer(const std::string window_name) : name_(window_name), stop_(false), thread_(boost::ref(*this))
  {
    runmutex_.lock();
  }

  ~Visualizer() {}

  void stop()
  {
    runmutex_.unlock();
    stop_ = true;
    thread_.join();
  }

  void operator()()
  {
	std::cout << "Here" << std::endl;
	PCLVisualizer viewer(name_);
	std::cout << "next" << std::endl;
  	boost::shared_ptr<const Cloud> cloud;
	std::string key = "main_cloud";

    while(!stop_)
	{
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));

	  data_mutex.lock();
	  cloud = pc_;
	  data_mutex.unlock();

	  if(!viewer.updatePointCloud(pc_, key))
	  {
		viewer.addPointCloud(pc_, key);
	  }

	  if(viewer.wasStopped())
		stop_ = true;
	}
  }

  void setPointCloud(boost::shared_ptr<const Cloud>& cloud)
  {
	boost::mutex::scoped_lock lock(data_mutex);
	pc_ = cloud;
  }

  bool stop_;
  std::string name_;
  boost::mutex runmutex_, data_mutex;
  boost::thread thread_;
  boost::shared_ptr<const Cloud> pc_;
};


namespace ecto {
  namespace pcl {
	struct CloudViewer2 {
	  static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
	  {
		inputs.declare<PointCloud> ("input", "An XYZ/XYZRGB point cloud");
	  }

	  static void
	  declare_params(tendrils& params)
	  {
		params.declare<std::string>("window_name", "The window name", "cloud viewer");
	  }

	  void
	  configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
	  {
		std::string window_name;
		params["window_name"] >> window_name;
		impl_.reset(new Visualizer(window_name));
		pc_ = inputs["input"];
	  }

      int process(const tendrils& inputs, const tendrils& outputs)
	  {
		boost::shared_ptr<const Cloud> c = pc_->cast<Cloud>();
		impl_->setPointCloud(c);
		return ecto::OK;
	  }

	  ~CloudViewer2()
	  {
        if(impl_)
	      impl_->stop();
        impl_.reset();
	  }

      boost::scoped_ptr<Visualizer> impl_;
      spore<PointCloud> pc_;
	  boost::shared_ptr<PCLVisualizer> viewer_;
	};
  }
}

ECTO_CELL(ecto_pcl, ecto::pcl::CloudViewer2, "CloudViewer2", "Cloud Visualizer");
