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

cs::ICameraPtr camera = nullptr;
int saveFlag = -1;

void getPairedFrameThread()
{
    using Clock = std::chrono::steady_clock;

	// 记录开始时间点
    auto start = Clock::now();

	while (true)
	{
		// 记录结束时间点
		auto end = Clock::now();

		// 计算时间差（自动推导为浮点型毫秒）
		auto duration_ms = std::chrono::duration<double, std::milli>(end - start);

		if (duration_ms.count() >= 10*000)
		{
			break;
		}

		cs::IFramePtr depthFrame = nullptr;
		cs::IFramePtr rgbFrame = nullptr;

		auto ret = camera->getPairedFrame(depthFrame, rgbFrame, 1000);
		if (ret == SUCCESS)
		{
			printf("get depth frame timestamp = %f, rgb frame timestamp = %f, D-value = %f\r\n", depthFrame->getTimeStamp(), rgbFrame->getTimeStamp(), (depthFrame->getTimeStamp() - rgbFrame->getTimeStamp()));
		}
	}
}

int main()
{
	printf("This is SampleDepthRGBFrameMatch sample!\n\n");

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
	camera = cs::getCameraPtr();

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
		if (streamInfo.format == STREAM_FORMAT_Z16)
		{
			//调用启动流接口
			ret = camera->startStream(STREAM_TYPE_DEPTH, streamInfo, NULL, NULL, 10);
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

	// 启动RGB相机流,选择需要的流格式
	// start rgb-stream 
	for (auto streamInfo : streamInfos)
	{
		if (streamInfo.format == STREAM_FORMAT_MJPG)
		{
			//调用启动流接口
			ret = camera->startStream(STREAM_TYPE_RGB, streamInfo, NULL, NULL);
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
	// 关闭深度自动曝光，如需要自动曝光，根据自动曝光类型选择
	PropertyExtension depthAutoExpProperty;
	depthAutoExpProperty.autoExposureMode = AUTO_EXPOSURE_MODE::AUTO_EXPOSURE_MODE_CLOSE;
	ret = camera->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, depthAutoExpProperty);
	if (ret != SUCCESS)
	{
		printf("camera set Depth auto exposure mode failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set Depth auto exposure mode done\n");
	}

	/**************************** 标准属性方式设置 ****************************/
	//调用标准属性获取深度相机曝光时间范围
	ret = camera->getPropertyRange(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, min, max, step);
	if (ret != SUCCESS)
	{
		printf("camera get Depth exposure range failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera get Depth exposure range done, min = %f, max = %f, step = %f\n", min, max, step);
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	//调用标准属性设置深度相机帧时间
	ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, 7500);
	if (ret != SUCCESS)
	{
		printf("camera set Depth frame time failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set Depth frame time done\n");
	}

	//调用标准属性设置深度相机曝光时间，注意曝光时间必须小于帧时间
	ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, 7000);
	if (ret != SUCCESS)
	{
		printf("camera set Depth exposure failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set Depth exposure done\n");
	}

	/**************************** 扩展属性方式设置 ****************************/
	// 设置深度与RGB的帧时间戳匹配参数
	PropertyExtension DepthRgbMatchParam;
	DepthRgbMatchParam.depthRgbMatchParam.bMakeSureRgbIsAfterDepth = false;		//控制是否强制RGB在深度之后，即RGB时间戳小于深度，如对顺序无要求填false
	DepthRgbMatchParam.depthRgbMatchParam.iDifThreshold = 60;					//深度与RGB的时间戳差值阈值，越小表示深度与RGB的时间越接近，但会影响帧率
	DepthRgbMatchParam.depthRgbMatchParam.iRgbOffset = 0;						//RGB时间戳偏移量，一般填0

	ret = camera->setPropertyExtension(PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM, depthAutoExpProperty);
	if (ret != SUCCESS)
	{
		printf("camera set Depth and rgb match param failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set Depth and rgb match param done\n");
	}

	//在线程里获取匹配后的帧数据
	std::thread(getPairedFrameThread).join();

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