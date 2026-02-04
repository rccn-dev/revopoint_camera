#include "3DCamera.hpp"
#include <fcntl.h>
#include <BmpUtil.hpp>

int main()
{
	printf( "This is SampleLiveRgbManualQuery sample!\n\n");
	ERROR_CODE ret;

	// get camera pointer and connect a valid camera
	cs::ICameraPtr camera = cs::getCameraPtr();
	ret = camera->connect();
	if(ret != SUCCESS)
	{
		printf("camera connect failed(%d)!\n", ret);
		return -1;
	}

	// get  informations of camera
	CameraInfo info;
	ret = camera->getInfo(info);
	if(ret != SUCCESS)
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
	
	// get  informations of rgb-stream 
	std::vector<StreamInfo> streamInfos;
	ret = camera->getStreamInfos(STREAM_TYPE_RGB,  streamInfos);
	if(ret != SUCCESS)
	{
		printf("camera get stream info failed(%d)!\n", ret);
		return -1;
	}

	// display  informations of depth-stream 
	for (auto streamInfo : streamInfos)
	{
		printf("depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
	}
	printf("\n");

	// start rgb-stream 
	for (auto streamInfo : streamInfos)
	{
		if (streamInfo.format == STREAM_FORMAT_RGB8)
		{
			ret = camera->startStream(STREAM_TYPE_RGB, streamInfo);
			if (ret != SUCCESS)
			{
				printf("camera start rgb stream failed(%d)!\n", ret);
				return -1;
			}
			break;
		}
		/*else if (streamInfo.format == STREAM_FORMAT_MJPG)
		{
			ret = camera->startStream(STREAM_TYPE_RGB, streamInfo);
			if (ret != SUCCESS)
			{
				printf("camera start rgb stream failed(%d)!\n", ret);
				return -1;
			}
			break;
		}*/
	}

	// capture and save 60 frames data in the live rgb-stream
	for(int i = 0; i < 60; i++)
	{
		cs::IFramePtr frame;
		ret = camera->getFrame(STREAM_TYPE_RGB, frame);
		if(ret != SUCCESS)
		{
			printf("camera get frame failed(%d)!\n", ret);
		}
		else
		{
			printf("Get a new rgb frame! width=%d height=%d size=%d\n", frame->getWidth(), frame->getHeight(), frame->getSize());
			if (frame->getFormat() == STREAM_FORMAT_MJPG)
			{
				FILE *fp = fopen("RgbManualQuery.jpg", "wb+");
				if (fp)
				{
					fwrite(frame->getData(), 1, frame->getSize(), fp);
					fclose(fp);
				}
			}
			else
			{
				cs::saveBmp(frame->getData(), frame->getWidth(), frame->getHeight(), 3, "RgbManualQuery.bmp");
			}
		}
	}

	// stop rgb-stream
	ret = camera->stopStream(STREAM_TYPE_RGB);
	if(ret != SUCCESS)
	{
		printf("camera stop rgb stream failed(%d)!\n", ret);
		return -1;
	}

	// disconnect camera
	ret = camera->disconnect();
	if(ret != SUCCESS)
	{
		printf("camera disconnect failed(%d)!\n", ret);
		return -1;
	}
	
	return 0;	
}

