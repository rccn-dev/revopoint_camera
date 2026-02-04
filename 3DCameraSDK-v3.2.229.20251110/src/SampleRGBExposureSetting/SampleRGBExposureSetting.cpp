/*******************************************************************************
 * This file is part of the 3D Camera API Sample
 *
 * Copyright 2015-2020 (C) Chishine3D AS
 * All rights reserved.
 *
 * Chishine3D Software License, v1.0
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Chishine3D AS nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * 4. This software, with or without modification, must not be used with any
 * other 3D camera than from Chishine3D AS.
 *
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY CHISHINE3D AS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CHISHINE3D AS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Contact: Chishine3D Support <support@chishine3d.com>
 * Info:    http://www.chishine3d.com
 ******************************************************************************/
#include "3DCamera.hpp"
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <BmpUtil.hpp>
#include <thread>

int saveFlag = -1;
// Callback of stream in format MJPG
void Callback(cs::IFramePtr frame, void* usrData)
{
	if (saveFlag >= 0)
	{
		char path[64];
		sprintf(path, "RGBLedCtrl%d.jpg", saveFlag++);
		cs::saveBmp(frame->getData(), frame->getWidth(), frame->getHeight(), 3, path);
		saveFlag = -1;
	}
	//printf("Get a new mjpg frame! It has saved as \'%s\'\n",path);
}

int main()
{
	printf("This is SampleRGBExposureSetting sample!\n\n");

	//cs::setLogSavePath("sdk.log");
	//cs::enableLoging(true);

	ERROR_CODE ret;

	cs::ISystemPtr system = cs::getSystemPtr();

	//枚举查询相机列表,选择一个相机
	//find a camera to connect
	CameraInfo info;
	while (true)
	{
		std::vector<CameraInfo> cameras;

		//调用主动查询相机接口
		ERROR_CODE ret = system->queryCameras(cameras);
		if (ret == SUCCESS && cameras.size() > 0)
		{
			info = cameras[0];
			printf("select the first camera serial:%s\n", info.serial);
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	// 获取相机共享指针
	// get camera pointer and connect a valid camera
	cs::ICameraPtr camera = cs::getCameraPtr();

	//连接指定信息相机
	//connect the camera of info
	ret = camera->connect(info);
	if (ret != SUCCESS)
	{
		printf("camera connect failed(%d)!\n", ret);
		return -1;
	}

	// display informations of camera
	printf("%20s  :  %s\n", "name", info.name);
	printf("%20s  :  %s\n", "serial", info.serial);
	printf("%20s  :  %s\n", "unique id", info.uniqueId);
	printf("%20s  :  %s\n", "firmware version", info.firmwareVersion);
	printf("%20s  :  %s\n", "algorithm version", info.algorithmVersion);
	printf("\n");

	// 获取相机支持的RGB流信息列表
	// get informations list of rgb-stream 
	std::vector<StreamInfo> streamInfos;
	ret = camera->getStreamInfos(STREAM_TYPE_RGB, streamInfos);
	if (ret != SUCCESS)
	{
		printf("camera get stream info failed(%d)!\n", ret);
		return -1;
	}

	// display  informations of rgb-stream 
	for (auto streamInfo : streamInfos)
	{
		printf("format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
	}
	printf("\n");

	// 获取相机支持的深度流信息列表
	// get informations list of depth-stream
	std::vector<StreamInfo> streamInfosDepth;
	ret = camera->getStreamInfos(STREAM_TYPE_DEPTH, streamInfosDepth);
	if (ret != SUCCESS)
	{
		printf("camera get stream info failed(%d)!\n", ret);
		return -1;
	}

	// display  informations of depth-stream 
	for (auto streamInfo : streamInfosDepth)
	{
		printf("format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
	}
	printf("\n");

	// 启动深度相机流,选择需要的流格式
	// start depth-stream 
	for (auto streamInfo : streamInfosDepth)
	{
		auto cameraType = cs::getCameraTypeBySN(info.serial);
		if (cameraType == CAMERA_POP_2 || \
			cameraType == CAMERA_MINI_NORMAL || \
			cameraType == CAMERA_RANGE_2 || \
			cameraType == CAMERA_MINI_2 || \
			cameraType == CAMERA_HANDY_LOOK_2_PRO || \
			cameraType == CAMERA_POP_3 || \
			cameraType == CAMERA_INSPIRE
		)
		{
			//启动RGB流前需要启动深度流
			//Before starting the RGB stream, it is necessary to start the depth stream
			if (streamInfo.format == STREAM_FORMAT_Z16)
			{
				//调用启动流接口
				ret = camera->startStream(STREAM_TYPE_DEPTH, streamInfo, NULL, NULL);
				if (ret != SUCCESS)
				{
					printf("camera start depth stream failed(%d)!\n", ret);
					return -1;
				}
				else
				{
					printf("start depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
				}
				break;
			}
		}
	}

	// 启动RGB相机流,选择需要的流格式
	// start rgb-stream 
	for (auto streamInfo : streamInfos)
	{
		if (streamInfo.format == STREAM_FORMAT_MJPG)
		{
			//调用启动流接口
			ret = camera->startStream(STREAM_TYPE_RGB, streamInfo, Callback, NULL);
			if (ret != SUCCESS)
			{
				printf("camera start rgb stream failed(%d)!\n", ret);
				return -1;
			}
			else
			{
				printf("start rgb format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
			}
			break;
		}
	}

	saveFlag = 0;
	std::this_thread::sleep_for(std::chrono::seconds(1));

	//调用标准属性关闭RGB相机自动曝光
	float auto_exposure = 0;
	ret = camera->setProperty(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_EXPOSURE, auto_exposure);
	if (ret != SUCCESS)
	{
		printf("camera set auto exposure mode failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set auto exposure mode %f\n");
	}

	std::this_thread::sleep_for(std::chrono::seconds(3));

	/**************************** 标准属性方式设置 ****************************/
	//调用标准属性获取RGB相机曝光时间范围,注意标准属性方式的值是无单位的，只是值大小体现
	float min, max, step;
	ret = camera->getPropertyRange(STREAM_TYPE_RGB, PROPERTY_EXPOSURE, min, max, step);
	if (ret != SUCCESS)
	{
		printf("camera get RGB exposure range failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera get RGB exposure range done, min = %f, max = %f, step = %f\n", min, max, step);
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	//调用标准属性设置RGB相机曝光时间,注意标准属性方式的值是无单位的，只是值大小体现， 确保设置的值是在获取的范围内，一般是-12~2，具体由获取到的为准
	ret = camera->setProperty(STREAM_TYPE_RGB, PROPERTY_EXPOSURE, 1);
	if (ret != SUCCESS)
	{
		printf("camera set RGB exposure failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set RGB exposure done\n");
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	// 获取RGB相机的曝光时间(标准属性方式)
	float expValue = 1.0;
	ret = camera->getProperty(STREAM_TYPE_RGB, PROPERTY_EXPOSURE, expValue);
	if (ret != SUCCESS)
	{
		printf("camera get RGB exposure failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera get RGB exposure done, exp = %f\n", expValue);
	}

	/**************************** 扩展属性方式设置 ****************************/
	// 获取RGB相机的曝光时间范围
	PropertyExtension rgbExposureRange;
	ret = camera->getPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB, rgbExposureRange);
	if (ret != SUCCESS)
	{
		printf("camera get RGB exposure range (extension) failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera get RGB exposure range (extension) done, min = %f, max = %f, step = %f\n", rgbExposureRange.objVRange_.fMin_, rgbExposureRange.objVRange_.fMax_, rgbExposureRange.objVRange_.fStep_);
	}

	//调用扩展属性设置RGB相机曝光时间,注意扩展属性方式的值单位是微秒， 确保设置的值是在获取的范围内
	PropertyExtension rgbExposureExtenstion;
	rgbExposureExtenstion.uiExposureTime = 5000;
	ret = camera->setPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RGB, rgbExposureExtenstion);
	if (ret != SUCCESS)
	{
		printf("camera set RGB exposure (extension) failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set RGB exposure (extension) done\n");
	}

	// 获取RGB相机的曝光时间(扩展属性方式)
	ret = camera->getPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RGB, rgbExposureExtenstion);
	if (ret != SUCCESS)
	{
		printf("camera get RGB exposure (extension) failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera get RGB exposure (extension) done, exp = %f ms\n", rgbExposureExtenstion.uiExposureTime);
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	// stop depth-stream
	ret = camera->stopStream(STREAM_TYPE_DEPTH);
	if (ret != SUCCESS)
	{
		printf("camera stop rgb stream failed(%d)!\n", ret);
		return -1;
	}

	// stop rgb-stream
	ret = camera->stopStream(STREAM_TYPE_RGB);
	if (ret != SUCCESS)
	{
		printf("camera stop rgb stream failed(%d)!\n", ret);
		return -1;
	}

	// disconnect camera
	ret = camera->disconnect();
	if (ret != SUCCESS)
	{
		printf("camera disconnect failed(%d)!\n", ret);
		return -1;
	}

	return 0;
}