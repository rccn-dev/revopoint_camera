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
		sprintf(path, "SampleProjecterCtrl%d.jpg", saveFlag++);
		cs::saveBmp(frame->getData(), frame->getWidth(), frame->getHeight(), 3, path);
		saveFlag = -1;
	}
	//printf("Get a new mjpg frame! It has saved as \'%s\'\n",path);
}

int main()
{
	printf("This is SampleProjecterCtrl sample!\n\n");

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
			ret = camera->startStream(STREAM_TYPE_DEPTH, streamInfo, Callback, NULL);
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

	saveFlag = 0;
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// 光机补光灯控制扩展属性
	// projector led ctrl property extension
	PropertyExtension pe;

	//选择激光器类型
	pe.ledCtrlParam.emLedId = LED_ID::LASER_LED;

	//选择使能或失能，关闭或打开，LED_CTRL_TYPE::ENABLE_LED、LED_CTRL_TYPE::DISABLE_LED
	pe.ledCtrlParam.emCtrlType = LED_CTRL_TYPE::ENABLE_LED;

	//调用扩展属性设置接口
	ret = camera->setPropertyExtension(PROPERTY_EXT_LED_CTRL, pe);
	if (ret != SUCCESS)
	{
		printf("camera set projecter ctrl failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set projecter ctrl %d\n", pe.ledCtrlParam.emCtrlType);
	}
	std::this_thread::sleep_for(std::chrono::seconds(2));

	//选择激光器类型
	pe.ledCtrlParam.emLedId = LED_ID::LASER_LED;

	// 选择激光器亮度调节控制
	pe.ledCtrlParam.emCtrlType = LED_CTRL_TYPE::LUMINANCE;

	// 设置激光器亮度，范围为0~255
	pe.ledCtrlParam.luminance = 100;

	//调用扩展属性设置接口
	ret = camera->setPropertyExtension(PROPERTY_EXT_LED_CTRL, pe);
	if (ret != SUCCESS)
	{
		printf("camera set projecter luminance ctrl failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set projecter luminance ctrl %d\n", pe.ledCtrlParam.emCtrlType);
	}

	std::this_thread::sleep_for(std::chrono::seconds(5));

	// stop depth-stream
	ret = camera->stopStream(STREAM_TYPE_DEPTH);
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