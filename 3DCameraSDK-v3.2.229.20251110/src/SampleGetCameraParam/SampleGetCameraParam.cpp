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

int main()
{
	printf("This is get camera param sample!\n\n");
	ERROR_CODE ret;

	// get camera pointer and connect a valid camera
	cs::ICameraPtr camera = cs::getCameraPtr();
	ret = camera->connect();
	if (ret != SUCCESS)
	{
		printf("camera connect failed(%d)!\n", ret);
		return -1;
	}

	// get  informations of camera
	CameraInfo info;
	ret = camera->getInfo(info);
	if (ret != SUCCESS)
	{
		printf("camera get info failed(%d)!\n", ret);
		return -1;
	}

	// display informations of camera
	printf("%20s  :  %s\n", "name", info.name);
	printf("%20s  :  %s\n", "serial", info.serial);
	printf("%20s  :  %s\n", "unique id", info.uniqueId);
	printf("%20s  :  %s\n", "firmware version", info.firmwareVersion);
	printf("%20s  :  %s\n", "algorithm version", info.algorithmVersion);
	printf("\n");

	// 深度缩放系数
	PropertyExtension pe;
	if (SUCCESS == camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, pe))
	{
		printf("camera depth scale %f\n", pe.depthScale);
	}
	printf("\n");

	// 相机内参
	Intrinsics intrRGB;
	if (SUCCESS == camera->getIntrinsics(STREAM_TYPE_RGB, intrRGB))
	{
		printf("camera RGB intrinsics.\n");
		printf("%20s  :  %d\n", "width", intrRGB.width);
		printf("%20s  :  %d\n", "height", intrRGB.height);
		printf("%20s  :  %f\n", "cx", intrRGB.cx);
		printf("%20s  :  %f\n", "cy", intrRGB.cy);
		printf("%20s  :  %f\n", "fx", intrRGB.fx);
		printf("%20s  :  %f\n", "fy", intrRGB.fy);
		printf("%20s  :  %f\n", "one22", intrRGB.one22);
		printf("%20s  :  %f\n", "zero01", intrRGB.zero01);
		printf("%20s  :  %f\n", "zero10", intrRGB.zeor10);
		printf("%20s  :  %f\n", "zeor20", intrRGB.zeor20);
		printf("%20s  :  %f\n", "zero21", intrRGB.zero21);
	}
	Intrinsics intrDepth;
	if (SUCCESS == camera->getIntrinsics(STREAM_TYPE_DEPTH, intrDepth))
	{
		printf("camera Depth intrinsics.\n");
		printf("%20s  :  %d\n", "width", intrDepth.width);
		printf("%20s  :  %d\n", "height", intrDepth.height);
		printf("%20s  :  %f\n", "cx", intrDepth.cx);
		printf("%20s  :  %f\n", "cy", intrDepth.cy);
		printf("%20s  :  %f\n", "fx", intrDepth.fx);
		printf("%20s  :  %f\n", "fy", intrDepth.fy);
		printf("%20s  :  %f\n", "one22", intrDepth.one22);
		printf("%20s  :  %f\n", "zero01", intrDepth.zero01);
		printf("%20s  :  %f\n", "zero10", intrDepth.zeor10);
		printf("%20s  :  %f\n", "zeor20", intrDepth.zeor20);
		printf("%20s  :  %f\n", "zero21", intrDepth.zero21);
	}
	printf("\n");

	// 深度RGB旋转平移矩阵参数
	Extrinsics entr;
	if (SUCCESS == camera->getExtrinsics(entr))
	{
		printf("camera extrinsics.\n");
		printf("rotation:");
		for (int i = 0; i < 9; i++)
		{
			printf(" %f", entr.rotation[i]);
		}
		printf("\n");
		printf("translation:");
		for (int i = 0; i < 3; i++)
		{
			printf(" %f", entr.translation[i]);
		}
		printf("\n");
	}
	printf("\n");

	// 畸变参数
	Distort distRGB;
	if (SUCCESS == camera->getDistort(STREAM_TYPE_RGB, distRGB))
	{
		printf("camera RGB distort.\n");
		printf("%20s  :  %f\n", "k1", distRGB.k1);
		printf("%20s  :  %f\n", "k2", distRGB.k2);
		printf("%20s  :  %f\n", "k3", distRGB.k3);
		printf("%20s  :  %f\n", "k4", distRGB.k4);
		printf("%20s  :  %f\n", "k5", distRGB.k5);
	}
	Distort distDepth;
	if (SUCCESS == camera->getDistort(STREAM_TYPE_DEPTH, distDepth))
	{
		printf("camera Depth distort.\n");
		printf("%20s  :  %f\n", "k1", distDepth.k1);
		printf("%20s  :  %f\n", "k2", distDepth.k2);
		printf("%20s  :  %f\n", "k3", distDepth.k3);
		printf("%20s  :  %f\n", "k4", distDepth.k4);
		printf("%20s  :  %f\n", "k5", distDepth.k5);
	}
	printf("\n");

	// 透视变换矩阵参数
	PerspectiveTransformationMatrix matrix;
	if (SUCCESS == camera->getPerspectiveTransformationMatrix(matrix))
	{
		printf("camera matrix.\n");
		printf("%20s  :  %d\n", "width", matrix.width);
		printf("%20s  :  %d\n", "height", matrix.height);
	}
	printf("\n");

	// disconnect camera
	ret = camera->disconnect();
	if (ret != SUCCESS)
	{
		printf("camera disconnect failed(%d)!\n", ret);
		return -1;
	}

	return 0;
}