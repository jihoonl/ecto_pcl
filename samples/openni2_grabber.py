#!/usr/bin/env python

"""
This sample shows how to grab openni2
"""

import sys
import ecto, ecto_ros, ecto_pcl_ros, ecto_pcl
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto.opts import scheduler_options, run_plasm
from ecto_pcl import *

def generate_graph():
    grabber = ecto_pcl.OpenNI2Grabber()

    grab = ecto_pcl.OpenNI2Grabber()
    cloud2msg = ecto_pcl_ros.PointCloud2Message("cloud2msg")
    cloud_pub = ecto_sensor_msgs.Publisher_PointCloud2("cloud_pub",topic_name='/ecto_pcl/sample_output')

    graph = [grab[:] >> cloud2msg[:],
             cloud2msg[:] >> cloud_pub[:]]

    return graph

if __name__ == "__main__":
    ecto_ros.init(sys.argv,"ecto_pcl_demo")

    plasm = ecto.Plasm()
    graph = generate_graph()
    plasm.connect(graph)
    ecto.view_plasm(plasm)
    ecto.opts.doit(plasm, description='Read a pcd through ROS.')
