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

int main()
{
	printf("This is SampleRedLightIndicator sample!\n\n");

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

	std::this_thread::sleep_for(std::chrono::seconds(1));

	// 红光指示灯控制扩展属性
	// red light indicator ctrl property extension
	PropertyExtension pe;

	//true-打开指示灯、false-关闭指示灯
	//true-turn on indicator、false-turn off indicator
	pe.redLightIndicator = true;

	//调用扩展属性设置接口
	ret = camera->setPropertyExtension(PROPERTY_EXT_RED_LIGHT_INDICATOR, pe);
	if (ret != SUCCESS)
	{
		printf("camera set red light indicator ctrl failed!(ret:%d)\n", ret);
	}
	else
	{
		printf("camera set red light indicator ctrl failed, %d\n", ret);
	}

	std::this_thread::sleep_for(std::chrono::seconds(2));

	// disconnect camera
	ret = camera->disconnect();
	if (ret != SUCCESS)
	{
		printf("camera disconnect failed(%d)!\n", ret);
		return -1;
	}

	return 0;
}