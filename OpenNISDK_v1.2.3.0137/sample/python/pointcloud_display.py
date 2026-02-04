# -*- coding: utf-8 -*-
# @File   : pointcloud_display.py
# @Date   : 2020/07/14
# @Author : PengLei
# @Mail   : penglei@chishine3d.com
# @bref   : This is a point cloud sample.
import ctypes
from openni import openni2
from openni import _openni2
import numpy as np
import cv2
import open3d
import csdevice as cs
    
if __name__ == '__main__':
    openni2.initialize()
    dev = openni2.Device.open_any()

    if dev.has_sensor(openni2.SENSOR_DEPTH) :
        stream = dev.create_stream(openni2.SENSOR_DEPTH)
        sensor_info = stream.get_sensor_info()
        stream.set_video_mode(sensor_info.videoModes[len(sensor_info.videoModes)-1])
        stream.start()

        intrinsics = stream.get_property(cs.CS_PROPERTY_STREAM_INTRINSICS, cs.Intrinsics)
        
        vis = open3d.visualization.Visualizer()
        vis.create_window()
        
        for count in range(100):
            frame = stream.read_frame()
            pc = cs.generatePointCloud(frame, intrinsics)
            
            if pc.any():
                pct = open3d.geometry.PointCloud()
                pct.points = open3d.utility.Vector3dVector(pc)
                vis.clear_geometries()
                vis.add_geometry(pct)
                vis.run() 
        
        vis.destroy_window()        
        stream.stop()
    else:
        print("Device does not have depth sensor!")
    dev.close()
