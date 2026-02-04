import ctypes
import numpy as np
from openni import _openni2

CS_PROPERTY_DEVICE_BASE             = (0xD0000000)
CS_PROPERTY_DEVICE_IR_MODE          = (CS_PROPERTY_DEVICE_BASE + 0x01)
CS_PROPERTY_DEVICE_EXTRINSICS       = (CS_PROPERTY_DEVICE_BASE + 0x02)

CS_PROPERTY_STREAM_BASE             = (0xE0000000)
CS_PROPERTY_STREAM_INTRINSICS       = (CS_PROPERTY_STREAM_BASE + 0x01)

class Distort(ctypes.Structure):
    _fields_ = [("k1", ctypes.c_float),
                ("k2", ctypes.c_float),
                ("k3", ctypes.c_float),
                ("k4", ctypes.c_float),
                ("k5", ctypes.c_float)]

    def __repr__(self):
        return 'Distort(k1 = %r, k2 = %r, k3 = %r, k4 = %r, k5 = %r)' % (
            self.k1, self.k2, self.k3, self.k4, self.k5)


class Intrinsics(ctypes.Structure):
    _fields_ = [("width", ctypes.c_short),
                ("height", ctypes.c_short),
                ("fx", ctypes.c_float),
                ("zero01", ctypes.c_float),
                ("cx", ctypes.c_float),
                ("zeor10", ctypes.c_float),
                ("fy", ctypes.c_float),
                ("cy", ctypes.c_float),
                ("zeor20", ctypes.c_float),
                ("zero21", ctypes.c_float),
                ("one22", ctypes.c_float)]

    def __repr__(self):
        return 'Intrinsics(width = %r, height = %r, fx = %r, fy = %r, cx = %r, cy = %r, zero01 = %r, zeor10 = %r, ' \
               'zeor20 = %r, zero21 = %r, one22 = %r)' % (
                   self.width, self.height, self.fx, self.fy, self.cx, self.cy, self.zero01, self.zeor10, self.zeor20,
                   self.zero21, self.one22)

class Extrinsics(ctypes.Structure):
    _fields_ = [("rotation", ctypes.c_float * 9),
                ("translation", ctypes.c_float * 3)]

    def __repr__(self):
        return 'Extrinsics(rotation = %r, translation = %r)' % (self.rotation, self.translation)


def generatePointCloud(frame, intrinsics):
    pc = []
    frame_data = np.array(frame.get_buffer_as_uint16()).reshape([frame.height, frame.width])
    if frame.videoMode.pixelFormat == _openni2.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM:
        depthScale = 0.1
    elif frame.videoMode.pixelFormat == _openni2.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM:
        depthScale = 1.0
    else:
        pc = np.array(pc)
        return pc
    
    fx = intrinsics.fx * frame.width / intrinsics.width
    fy = intrinsics.fy * frame.height / intrinsics.height
    cx = intrinsics.cx * frame.width / intrinsics.width
    cy = intrinsics.cy * frame.height / intrinsics.height
    
    
    for v in range(frame.height):
        for u in range(frame.width):
            if frame_data[v, u] > 0:
                z = frame_data[v, u] * depthScale
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                pc.append([x, y, z])
    
    pc = np.array(pc)
    return pc
    

