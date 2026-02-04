 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     Types.hpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __TYPES_HPP__
#define __TYPES_HPP__

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

	typedef struct CS_SDK_VERSION_TAG
	{			
		const char*	version;		//版本号
		const char*	name;			//模块名称
		const char*	author;			//创建者
		const char*	date;			//创建日期
		const char*	desc;			//描述，每次修改都需要描述
	}CS_SDK_VERSION, PCS_SDK_VERSION;

	//ProbeType
	typedef enum
	{
		PT_START,
		PT_STOP
	}ProbeType;

	/**
	 * @~chinese
	 * @brief	IMU标定平面
	 * @~english
	 * @brief	IMU CalibrationPlane
	 */
	typedef enum {
		IMU_XY = 0,			//XY平面
		IMU_XZ = 1,			//XZ平面
		IMU_YZ = 2,			//YZ平面
	}IMU_CALIBRATTION_PLANE;

	/**
	 * @~chinese
	 * @brief	IMU标定状态
	 * @~english
	 * @brief	IMU Calibration state
	 */
	typedef enum {
		CALIBRATION_STATE_ING,			//未完成
		CALIBRATION_STATE_FIN			//已完成
	}IMU_CALIBRATTION_STATE;
	
	/**
	* imu 数据
	**/
#pragma pack(push, 1) // 开始一个对齐设置的保存栈，1 字节对齐
	typedef struct
	{
		unsigned long long timestamp;
		char magic[3];
		float gx;
		float gy;
		float gz;

		float ax;
		float ay;
		float az;

		unsigned int rsv;

		unsigned short frame_count;
		unsigned char  crc;
	}imu_data_t;
#pragma pack(pop) 
	/**
	 * @~chinese
	 * @brief	LED标识
	 * @~english
	 * @brief	LED ID
	 */
	typedef enum {
		IR_LED,				//IR LED
		RGB_LED,			//RGB LED
		LASER_LED,			//激光器 LED
	}LED_ID;

	/**
	 * @~chinese
	 * @brief	LED控制类型
	 * @~english
	 * @brief	LED contrl type
	 */
	typedef enum {
		OFTEN_BRIGHT_LED,	//常亮
		TWINKLE_LED,		//闪烁
		ENABLE_LED,			//使能LED
		DISABLE_LED,		//失能LED
		LUMINANCE,			//调节亮度
		TIMES_CTRL,			//设置IR补光灯时间
		ILLUMINE_MOMENT,	//补光灯点亮时机
	}LED_CTRL_TYPE;

	/**
	 * @~chinese
	 * @brief	LED激光器控制支持
	 * @~english
	 * @brief	LED contrl is support
	 */
	typedef struct LED_CTRL_IS_SUPPORT {
		LED_ID ledId;			//LED激光机类型
		LED_CTRL_TYPE ctrlType;	//控制类型
		bool isSupport;			//是否支持控制
	}LED_CTRL_IS_SUPPORT;

	//CameraType
	typedef enum
	{
		CAMERA_UNKNOW,				//unknow camera type
		CAMERA_HD,					//HD
		CAMERA_SURFACE,				//surface
		CAMERA_SENSE_PRO,			//sense pro
		CAMERA_POP_1,			    //pop1
		CAMERA_POP_2 = 10,		    //pop2
		CAMERA_MINI_NO_RGB,			//mini camera without rgb sensor
		CAMERA_MINI_NORMAL,			//mini camera normal
        CAMERA_TRACER_P1,           //tracer p1
		CAMERA_RANGE,				//Range
		CAMERA_TRACER_P2,           //tracer p2
		CAMERA_POP_3,               //pop3
		CAMERA_INSPIRE,             //inspire
		CAMERA_RANGE_2,				//Range2
		CAMERA_MINI_2,				//Mini2
		CAMERA_MIRACO,				//Miraco
		CAMERA_HANDY_LOOK_2_PRO,	//Handy look 2 pro
		CAMERA_METRO_X,				//MetroX
		CAMERA_TRACKIT_SCANNER,		//Scanner for Trackit
		CAMERA_TRACKIT_TRACKER,		//Tracker for Trackit
		CAMERA_TRACER_R1,           //tracer r1
		CAMERA_INSPIRE_2,           //inspire2
		CAMERA_SURFACE_PRO_50,		//surface pro 50
		CAMERA_SURFACE_PRO_100,		//surface pro 100
		CAMERA_METRO_Y,				//MetroY
		CAMERA_METRO_Y_PRO,			//MetroY Pro
		CAMERA_METRO_X_PRO,			//MetroX Pro
	}CameraType;

#define FRAMERATE_ANY 0
#define CAMERA_SERIAL_MAX		32
#define FIRMWARE_VERSION_MAX	32
#define ALGORITHM_VERSION_MAX	32

#ifndef OUT
#define OUT
#endif

#ifndef IN
#define IN
#endif

typedef enum CAMERA_STATUS
{
	CS_NONE,			/**< @~chinese 无效的相机             @~english invalid camera*/
	CS_IDLE,			/**< @~chinese 空闲中             @~english in idle status*/
	CS_CONNECTED_BY_SDK,		/**< @~chinese 本SDK连接中             @~english current sdk connect to the camera.*/
	CS_CONNECTED_BY_OTHER,		/**< @~chinese 其他实例(进程)连接中             @~english other instance connect to the camera*/
}CAMERA_STATUS;

typedef enum ParamMngType
{
	RECOVERY_COPY = 0,			/**< @~chinese 还原参数文件						@~english reovery param file from backup*/
	GET_BACKUP_FILE,			/**< @~chinese 获取备份参数文件					@~english reovery param file from backup*/
	GET_CUUR_FILE,				/**< @~chinese 获取当前参数文件					@~english current param file from backup*/
	SET_BACKUP_FILE,			/**< @~chinese 设置备份参数文件					@~english reovery param file from backup*/
	SET_CURR_FILE,				/**< @~chinese 设置当前参数文件					@~english current param file from backup*/
	QUERY_BACKUP_STATE,			/**< @~chinese 查询是否有备分参数				@~english query backup state*/
}ParamMngType;

/**
* @~chinese
* 相机参数管理
* @~english
* enumeration: returned error code
**/
typedef struct CameraParamMng {
	ParamMngType mngType;				/**< @~chinese 参数文件管理类型				@~english param file mng type*/
	char paramFilePath[1024];			/**< @~chinese 参数文件所在路径             @~english Path of param file*/
} CameraParamMng;

/**
* @~chinese
* 枚举: 返回的错误码
* @~english
* enumeration: returned error code
**/
typedef enum ERROR_CODE
{
	SUCCESS = 0,					/**< @~chinese 成功             @~english success*/
	ERROR_PARAM,					/**< @~chinese 参数输入错误     @~english param input error*/
	ERROR_DEVICE_NOT_FOUND,			/**< @~chinese 未找到设备       @~english device not found*/
	ERROR_DEVICE_NOT_CONNECT,		/**< @~chinese 设备未连接       @~english device not connected*/
	ERROR_DEVICE_BUSY,				/**< @~chinese 设备忙           @~english device busy*/
	ERROR_STREAM_NOT_START = 5,		/**< @~chinese 流尚未打开       @~english stream not start*/
	ERROR_STREAM_BUSY,				/**< @~chinese 流已打开         @~english stream had started*/
	ERROR_FRAME_TIMEOUT,			/**< @~chinese 获取帧数据失败   @~english get frame failed*/
	ERROR_NOT_SUPPORT,				/**< @~chinese 尚不支持         @~english not support*/
	ERROR_PROPERTY_GET_FAILED,		/**< @~chinese 获取属性失败     @~english get property failed*/
	ERROR_PROPERTY_SET_FAILED = 10,	/**< @~chinese 设置属性失败     @~english set property failed*/
	ERROR_HID_CHANNEL_ERROR,		/**< @~chinese hid 通道异常	    @~english hid channel error*/
	ERROR_HID_WRITE_ERROR,			/**< @~chinese hid 通道写异常	@~english hid channel write error*/
	ERROR_HID_READ_ERROR,			/**< @~chinese hid 通道读异常	@~english hid channel read error*/
	ERROR_DYNAMIC_RES_ABSENCE,		/**< @~chinese 动态库缺失		@~english dynamic resource absence*/
	ERROR_DYNAMIC_FUC_ABSENCE,		/**< @~chinese 动态库功能缺失	@~english dynamic function absence*/
	ERROR_DEVICE_PARAM_ERROR,		/**< @~chinese 设备参数错误		@~english device param error*/
	ERROR_GET_INTERNAL_RES_FAILED,	/**< @~chinese 获取相机参数失败	@~english get camera param failed*/
	ERROR_UNKNOW,					/**< @~chinese 未知异常			@~english unknow error*/
}ERROR_CODE;

/**
* @~chinese
* 枚举: 相机流类型
* @~english
* enumeration: stream type
**/
typedef enum STREAM_TYPE
{
	STREAM_TYPE_DEPTH	= 0, /**<@~chinese 深度流  @~english Depth camera stream */
    STREAM_TYPE_RGB		= 1, /**<@~chinese RGB流   @~english RGB camera stream */
	STREAM_TYPE_COUNT
}STREAM_TYPE;


/// \~chinese
/// \defgroup StreamFormat 数据流格式
/// \brief 深度流和RGB流所支持的所有格式
/// @{
/// \~english
/// \defgroup StreamFormat Stream format
/// \brief Format of depth stream and RGB stream
/// @{dump
/**
* @~chinese
* 枚举: 流数据格式
* @~english
* enumeration: stream format
**/
typedef enum STREAM_FORMAT
{
	STREAM_FORMAT_MJPG		= 0x00,			/**< @~chinese RGB流的MJPG压缩的数据			
											  @~english MJPG compressed data*/ 
	STREAM_FORMAT_RGB8		= 0x01,			/**< @~chinese RGB流的8位红,绿,蓝3通道数据			
											  @~english 8-bit red, green and blue channels*/ 
	STREAM_FORMAT_Z16		= 0x02,			/**< @~chinese 深度流的深度图格式, 每一个深度值以unsigned short表示
											  @~english 16-bit unsigned short depth values. The depth in millimeters is equal to depth scale * pixel value. */ 
	STREAM_FORMAT_Z16Y8Y8	= 0x03,			/**< @~chinese 深度流的深度图+红外图组合格式,	
														通过FRAME_DATA_FORMAT_Z16获得深度数据，
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图, 
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
											  @~english output depth map and infrared, 
														get depth map by FRAME_DATA_FORMAT_Z16
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/ 
	STREAM_FORMAT_PAIR		= 0x04,			/**< @~chinese 深度流的红外图格式，适用于双目相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图, 
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图	
											  @~english output infrared，suitable for binocular camera
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/ 
	STREAM_FORMAT_H264		= 0x05,			/**< @~chinese RGB流的H264压缩的数据			
											  @~english H264 compressed data*/ 

	STREAM_FORMAT_RGBA		= 0x06,			/**< @~chinese RGBA流的红,绿,蓝,alpha(0xff) 4通道数据
											@~english RGBA data, red, green, blue and alpha(0xff) channels*/

	STREAM_FORMAT_I8DS      = 0x100,		/**< @~chinese 降采样红外预览格式
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图,
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图，
														通过FRAME_DATA_FORMAT_VCENTER_LEFT获得左红外图中光刀中心的y坐标数组,
														通过FRAME_DATA_FORMAT_VCENTER_RIGHT获得右红外图中光刀中心的y坐标数组，
														通过FRAME_DATA_FORMAT_LASER_WIDTH_LEFT 获得左红外图光刀宽度数组,
														通过FRAME_DATA_FORMAT_LASER_WIDTH_RIGHT获得右红外图光刀宽度数组
									    	  @~english output down sampled infrared for preview
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT, 
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT,
														get left laser center by FRAME_DATA_FORMAT_VCENTER_LEFT,
														get right laser center by FRAME_DATA_FORMAT_VCENTER_RIGHT,
														get left laser width by FRAME_DATA_FORMAT_LASER_WIDTH_LEFT,
														get right laser width by FRAME_DATA_FORMAT_LASER_WIDTH_RIGHT*/
	STREAM_FORMAT_XZ32		= 0x101,		/**< @~chinese 点云输出格式, 通过cs::Pointcloud::generatePointsFromXZ计算成点云		
											  @~english output point cloud, call cs::Pointcloud::generatePointsFromXZ to compute a point cloud*/
	STREAM_FORMAT_GRAY		= 0x102,		/**< @~chinese 深度流的红外图格式，适用于单目相机
											  @~english output infrared，suitable for monocular cameras */ 

	STREAM_FORMAT_XYZS		= 0x103,

	STREAM_FORMAT_PMLD		= 0x104,		/**< @~chinese 深度流的红外图标记点及点云生成格式，适用于多线激光相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图, 
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图	
											  @~english output infrared，Suitable for cross line laser cameras
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/

	STREAM_FORMAT_PSLD		= 0x105,				/**< @~chinese 深度流的红外图标记点及点云生成格式，适用于单线激光相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图,
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
												@~english output infrared，Suitable for single line laser cameras
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/

	STREAM_FORMAT_PPLD		= 0x106,				/**< @~chinese 深度流的红外图标记点及点云生成格式，适用于平行线激光相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图,
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
												@~english output infrared，Suitable for parallel line laser cameras
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/

	STREAM_FORMAT_PRCZ		= 0x107,			/**< @~chinese 深度流的压缩红外图标记点及点云生成信息格式，适用于一字线相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图,
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
											  @~english output infrared，Suitable for cameras that support single line light
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/

	STREAM_FORMAT_PMCZ		= 0x108,			/**< @~chinese 深度流的mask压缩红外图标记点及点云生成信息格式，适用于支持mask方式压缩Pair流的相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图,
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
											  @~english output infrared，Suitable for cameras that support mask based compression of Pair streams
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/

	STREAM_FORMAT_Z16Y8Y8P	= 0x109,			/**< @~chinese 深度流的深度图+极线校正红外图+原始左右红外图组合格式,
														通过FRAME_DATA_FORMAT_Z16获得深度数据，
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图，
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图，
														通过FRAME_DATA_FORMAT_IR_LEFT_ORIGINAL获得原始左红外图，
														通过FRAME_DATA_FORMAT_IR_RIGHT_ORIGINAL获得原始右红外图，
											  @~english output Combination format of depth map of depth flow+epipolar corrected infrared map+original left and right infrared map,
														get depth map by FRAME_DATA_FORMAT_Z16
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT,
														get original left infrared by FRAME_DATA_FORMAT_IR_LEFT_ORIGINAL,
														get original right infrared by FRAME_DATA_FORMAT_IR_RIGHT_ORIGINAL*/

	STREAM_FORMAT_CPAIR		= 0x10A,			/**< @~chinese 压缩方式的深度流的红外图格式，适用于双目相机
														通过FRAME_DATA_FORMAT_IR_LEFT获得左红外图,
														通过FRAME_DATA_FORMAT_IR_RIGHT获得右红外图
											  @~english output infrared image format of compressed，suitable for binocular camera
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/

	STREAM_FORMAT_COUNT
}STREAM_FORMAT;

/**
* @~chinese
* 枚举: 帧数据格式，用于获取复合流数据中的指定格式数据起始地址
* @~english
* enumeration: format of frame data, used for get specified data in a composite frame
**/
typedef enum FRAME_DATA_FORMAT
{
	FRAME_DATA_FORMAT_Z16				= 0x00,		/**< @~chinese 深度流的深度图格式, 每一个深度值以unsigned short表示
														 @~english 16-bit unsigned short depth values. The depth in millimeters is equal to depth scale * pixel value. */ 
	FRAME_DATA_FORMAT_IR_LEFT			= 0x01,		/**< @~chinese 左红外图数据， 8-bit unsigned char表示一个灰度值				
														 @~english 8-bit unsigned char gray level of left infrared*/
	FRAME_DATA_FORMAT_IR_RIGHT			= 0x02,		/**< @~chinese 右红外图数据， unsigned char表示一个灰度值				
														 @~english 8-bit unsigned char gray level of right infrared*/
	FRAME_DATA_FORMAT_VCENTER_LEFT		= 0x03,		/**< @~chinese 左红外图光刀中心， unsigned char表示一个y坐标				
														 @~english 8-bit unsigned char y coordinate of left laser center*/
	FRAME_DATA_FORMAT_VCENTER_RIGHT		= 0x04,		/**< @~chinese 右红外图光刀中心， unsigned char表示一个y坐标				
														 @~english 8-bit unsigned char y coordinate of right laser center*/
	FRAME_DATA_FORMAT_LASER_WIDTH_LEFT	= 0x05,		/**< @~chinese 左红外图光刀宽度， unsigned char表示一个宽度值, 光刀太宽时影响点的精确度				
														 @~english 8-bit unsigned char left laser width array，The accuracy of the point is affected when the laser is too wide*/
	FRAME_DATA_FORMAT_LASER_WIDTH_RIGHT = 0x06,		/**< @~chinese 右红外图光刀宽度， unsigned char表示一个宽度值, 光刀太宽时影响点的精确度					
														 @~english 8-bit unsigned char right laser width array，The accuracy of the point is affected when the laser is too wide*/
	FRAME_DATA_FORMAT_IR_LEFT_ORIGINAL	= 0x07,		/**< @~chinese 原始左红外图数据，仅在Z16Y8Y8P格式下适用， 8-bit unsigned char表示一个灰度值
														 @~english 8-bit unsigned char gray level of left infrared of original, only in Z16Y8Y8P format*/
	FRAME_DATA_FORMAT_IR_RIGHT_ORIGINAL = 0x08,		/**< @~chinese 原始右红外图数据，仅在Z16Y8Y8P格式下适用， unsigned char表示一个灰度值
														 @~english 8-bit unsigned char gray level of right infrared original, only in Z16Y8Y8P format*/
}FRAME_DATA_FORMAT;
/// @}

/// \~chinese
/// \defgroup PropertyType 基础属性
/// \brief 列举所有影响相机图像可设置的基础属性
/// @{
/// \~english
/// \defgroup PropertyType Basic property
/// \brief List all the basic attributes that can be set to affect camera images
/// @{

/**
* @~chinese
* 枚举: 相机的基本属性，每个类型的值可用范围通过getPeropertyRange接口获取，通过setProperty/getProperty接口设置/获取值
* @~english
* enumeration: basic property of camera, The available range of values for each type is obtained through the getPerortyRange interface, using setProperty/getProperty
**/
typedef enum PROPERTY_TYPE
{
    PROPERTY_GAIN                       = 0x00,	/**<@~chinese 增益，调节相机亮度,提高增益会引入噪声,导致深度精度下降。调节范围1~16, 高精度时,建议增益≤3
                                                    @~english gain of depth camera or RGB camera*/
    PROPERTY_EXPOSURE                   = 0x01,	/**<@~chinese 曝光值（windows下的值范围有负数，如-12~2，如需毫秒单位的使用扩展属性），用于调节相机亮度,数值越大亮度越高,不会引入噪声,但是帧率会随着曝光时间增大而降低,调节范围3000~60000微秒
                                                    @~english Controls exposure time of depth camera or RGB camera, The value range under Windows has negative numbers that Like -12~2, To use PropertyExtension in milliseconds*/
	PROPERTY_FRAMETIME					= 0x02,	/**<@~chinese 帧时间            @~english Frame time of depth camera */
	PROPERTY_FOCUS						= 0x03,	/**<@~chinese 焦距              @~english Focus of RGB camera*/
	PROPERTY_ENABLE_AUTO_FOCUS			= 0x04,	/**<@~chinese 是否自动对焦      @~english Enable / disable auto-focus of RGB camera*/
	PROPERTY_ENABLE_AUTO_EXPOSURE		= 0x05, /**<@~chinese 是否自动曝光      @~english Enable / disable auto-exposure of RGB camera*/
	PROPERTY_ENABLE_AUTO_WHITEBALANCE	= 0x06, /**<@~chinese 是否自动白平衡    @~english White balance of RGB camera*/
	PROPERTY_WHITEBALANCE				= 0x07,	/**<@~chinese 白平衡值          @~english adjust white balance of RGB camera*/
	PROPERTY_WHITEBALANCE_R				= 0x08,	/**<@~chinese 白平衡R通道       @~english Channel r of RGB camera, adjust white balance*/
	PROPERTY_WHITEBALANCE_B				= 0x09,	/**<@~chinese 白平衡B通道       @~english Channel b of RGB camera, adjust white balance*/
	PROPERTY_WHITEBALANCE_G				= 0x10,	/**<@~chinese 白平衡G通道       @~english Channel g of RGB camera, adjust white balance*/

} PROPERTY_TYPE;
/// @}

/**
* @~chinese
* 枚举: 相机触发模式 
* @~english
* enumeration: trigger mode
**/
typedef enum TRIGGER_MODE
{
	TRIGGER_MODE_OFF		= 0, /**< @~chinese 关闭触发模式，持续输出深度流	
									  @~english output depth map continuously*/ 
	TRIGGER_MODE_HARDWAER	= 1, /**< @~chinese 外触发模式，需要在触发口输入硬件信号才能出图
									  @~english external trigger mode,you should input hardware pulse to get depth frame*/
	TRIGGER_MODE_SOFTWAER	= 2, /**< @~chinese 软触发模式，需要调用cs::ICamera::softTrigger才能出深度图
									  @~english software trigger mode,you should call cs::ICamera::softTrigger to get depth frame*/
}TRIGGER_MODE;

/**
* @~chinese
* 枚举: 高动态的模式
* @~english
* enumeration: mode of HDR
**/
typedef enum HDR_MODE
{
	HDR_MODE_OFF			= 0,	/**< @~chinese 关闭				
                                         @~english HDR off*/ 
	HDR_MODE_HIGH_RELECT	= 1,	/**< @~chinese 高反模式适用于测高反物体,	
                                                   会按照设定的曝光级数,增加较低的曝光参数进行多次曝光后进行融合输出。曝光级数由用户设定
                                         @~english suitable for shiny object*/
	HDR_MODE_LOW_RELECT		= 2,	/**< @~chinese 暗色模式适用于测深色物体,	
                                                   会按照设定的曝光级数,增加较高曝光参数进行几次曝光后进行融合输出。曝光级数由用户设定
                                         @~english suitable for dark object*/
	HDR_MODE_ALL_RELECT		= 3		/**< @~chinese 复合模式适用于测复合表面	
                                                   会平均分配曝光级数,分别增加较低曝光参数和较高曝光参数来进行多次曝光并融合输出。曝光级数由用户设定
                                         @~english suitable for composite object*/
}HDR_MODE;

/**
* @~chinese
* @brief 枚举: 自动曝光模式
* @~english
* @brief enumeration: mode of auto exposure
**/
typedef enum AUTO_EXPOSURE_MODE
{
	AUTO_EXPOSURE_MODE_CLOSE = 0,			/**< @~chinese 关闭				
                                                 @~english off*/
	AUTO_EXPOSURE_MODE_FIX_FRAMETIME = 1,	/**< @~chinese 固定帧率模式：在当前帧率允许范围下自动实时调节相机的曝光时间和增益, 不调节相机的输出帧率
											     @~english adjust exposure automatically and keep frame time unchanged*/
	AUTO_EXPOSURE_MODE_HIGH_QUALITY = 2,	/**< @~chinese 高质量模式：以获得最高质量模型为目标, 自动实时调节相机的曝光时间和增益, 同时会按需调节帧率
											     @~english Highest quality model Priority, adjust exposure and frame time automatically*/
    AUTO_EXPOSURE_MODE_FORE_GROUND = 3		/**< @~chinese 近景优先模式：以近景为目标, 自动实时调节相机的曝光时间和增益, 同时会按需调节帧率
                                                 @~english Close Range Priority, adjust exposure and frame time automatically*/
}AUTO_EXPOSURE_MODE;

/**
* @~chinese
* @brief 枚举: 网络传输压缩方式
* @~english
* @brief enumeration: mode of compress
**/
typedef enum NETWORK_COMPRESS_MODE
{
	NETWORK_COMPRESS_MODE_CLOSE = 0,		/**< @~chinese 关闭               @~english off*/
	NETWORK_COMPRESS_MODE_ZIP	= 1,		/**< @~chinese ZIP(默认设置)      @~english ZIP(Default)*/
}NETWORK_COMPRESS_MODE;


/**
* @~chinese
* @brief 测量深度范围，超出范围的值将被置零
* @~english
* @brief range of depth, value out of range will be set to zero
**/
typedef struct DepthRange
{
	int min;		/**< @~chinese 深度最小值        @~english minimum of depth*/ 
	int max;		/**< @~chinese 深度最大值        @~english maximum of depth*/ 
}DepthRange;

/**
* @~chinese
* @brief 网络连接时设备的IP设置，当autoEnable设置为true时，无需设置ipFourthByte
* @~english
* @brief IP setting, when autoEnable is true, there is no need to set ipFourthByte
**/
typedef struct IpSetting
{
	unsigned int autoEnable;	/**< @~chinese 是否开启DHCP             @~english enable/disable DHCP*/ 
	unsigned char ipFourthByte;	/**< @~chinese IP地址的第四位           @~english the fourth byte of ip*/ 
}IpSetting;

typedef struct CameraIpSetting
{
    unsigned char autoEnable;	/**< @~chinese 是否开启DHCP             @~english enable/disable DHCP*/
    unsigned char ipBytes[4];	/**< @~chinese IP地址 {192,168,3,99}    @~english the first byte of ip*/
}CameraIpSetting;

/**
 * @~chinese
 * @brief	联网模式
 * @~english
 * @brief	net working mode
 */
typedef enum {
	NET_WORKING_TERMINAL,			//终端模式
	NET_WORKING_WIFI_HOST_AP,		//5G AP 模式，默认是5G模式
	NET_WORKING_WIFI_HOST_2_4G,		//2.4G AP模式，2.4G模式不开放，限内部使用
	NET_WORKING_TERMINAL_ONLY,		//终端模式并关闭AP
	NET_WORKING_WIFI_HOST_5GONLY,	//AP模式并关闭终端
}NET_WORKING_MODE;

#pragma pack(push, 4)
typedef enum WifiChannel
{
	WIFI_CHANNEL_DEFAULT = 0,	//默认信道，内部转换为WIFI_CHANNEL_44
	WIFI_CHANNEL_44 = 44,
	WIFI_CHANNEL_149 = 149
}WifiChannel;

/**
 * @~chinese
 * @brief		联网参数信息,对应命令PROPERTY_EXT_NET_WORKING_INFO
 * @~english
 * @brief		net working info,corresponding PROPERTY_EXT_NET_WORKING_INFO
 */
typedef struct
{
	NET_WORKING_MODE	netMode;
	unsigned char		cEnable;		//0:disable,1:enable
	char 				reserved[3];	//reserved
	char 				ssid[100];		//ap name or wifi host name
	char 				psk[100];		//password 
	WifiChannel			channel;		//WIFI通道
}NetworkingInfo;

#pragma pack(pop)

/**
* @~chinese
* @brief HDR自动模式时曝光级数及两级曝光之间的倍数设置
* @~english
* @brief exposure times and interstage scale of HDR
**/
typedef struct HdrScaleSetting
{
	unsigned int highReflectModeCount;	/**< @~chinese 高反模式曝光级数         @~english exposure times of high-reflective mode*/ 
	unsigned int highReflectModeScale;	/**< @~chinese 高反模式两级间倍数       @~english interstage scale of high-reflective mode*/ 
	unsigned int lowReflectModeCount;	/**< @~chinese 深色模式曝光级数         @~english exposure times of low-reflective mode*/ 
	unsigned int lowReflectModeScale;	/**< @~chinese 深色模式两级间倍数       @~english interstage scale of low-reflective mode*/ 
}HdrScaleSetting;

#pragma pack(push, 1)

/**
* @~chinese
* @brief HDR某一级曝光的参数
* @~english
* @brief exposure param of HDR
**/
typedef struct HdrExposureParam
{
	unsigned int  exposure;	/**< @~chinese 曝光时间         @~english exposure time*/ 
	unsigned char gain;		/**< @~chinese 增益             @~english gain*/ 
}HdrExposureParam;

/**
* @~chinese
* @brief HDR曝光参数
* @~english
* @brief all exposure params of HDR
**/
typedef struct HdrExposureSetting
{
	unsigned char count;			/**< @~chinese 总曝光级数        @~english total exposure times of HDR*/ 
	HdrExposureParam param[11];		/**< @~chinese 各级曝光参数      @~english all params of HDR*/ 
}HdrExposureSetting;

/**
* @~chinese
* @brief HDR某一级的曝光时间及光机亮度设置
* @~english
* @brief exposure and laser level of HDR
**/
typedef struct HdrBrightnessParam
{
	unsigned char laserLevel;		/**< @~chinese 激光器等级        @~english brightness of laser*/ 
	float exposure;					/**< @~chinese 曝光时间          @~english exposure time*/ 
}HdrBrightnessParam;

/**
* @~chinese
* @brief HDR所有级别的曝光时间及光机亮度设置
* @~english
* @brief all settings of exposure and laser level
**/
typedef struct HdrBrightnessSetting
{
	unsigned char count;			/**< @~chinese 总曝光级数        @~english total exposure times of HDR*/
	HdrBrightnessParam param[11];	/**< @~chinese 各级曝光参数      @~english all params of HDR*/
}HdrBrightnessSetting;

/**
* @~chinese
* @brief 深度和RGB帧匹配参数
* @~english
* @brief depth and rgb match param ,reference DepthRgbMatchParam
**/
typedef struct DepthRgbMatchParam
{
	int		iRgbOffset;				/**< @~chinese 匹配时RGB时间戳的偏移值     @~english the offset of rgb frame's timestamp when matching*/
	int		iDifThreshold;			/**< @~chinese 深度与RGB时间戳的误差阈值   @~english the threshold of depth and rgb frame's timestamp*/
	bool	bMakeSureRgbIsAfterDepth;	/**< @~chinese 确保RGB时间戳在深度之后   @~english make sure rgb's timestamp is after depth.*/
}DepthRgbMatchParam;

//条纹pattern切换前后需要stop_stream[0x483]和start_stream[0x481],sdk中已经处理好了，外部不需要重复调用483和481
//POP2的算法在1和3切换，当标志点模式时使用1，当其它模式时使用3--彭磊
//EN_FRINGE_PATTERN_TYPE_3FREQ4STEP_MULTI是18帧高帧率
const int id_msg_set_fringe_pattern_t = 0xa05;
typedef enum EN_FRINGE_PATTERN_TYPE_T 
{
	EN_FRINGE_PATTERN_TYPE_STANDARD = 0x00,							/**< @~chinese 标准4频4步相移模式   @~english */
	EN_FRINGE_PATTERN_TYPE_STANDARD_WHITE_ADDED = 0x01,				/**< @~chinese 白图+标准4频4步相移模式，用于标记点模式   @~english */
	EN_FRINGE_PATTERN_TYPE_MULTI_DEPTH = 0x02,						/**< @~chinese 4频4步相移+高频出2帧深度图模式   @~english */
	EN_FRINGE_PATTERN_TYPE_MULTI_DEPTH_WHITE_ADDED = 0x03,			/**< @~chinese 白图+4频4步相移+高频出2帧深度图模式，用于特征模式   @~english */
	EN_FRINGE_PATTERN_TYPE_MULTI_DEPTH_DOUBLE_WHITE_ADDED = 0x04,	/**< @~chinese 白图+4频4步相移+白图+高频出2帧深度图模式   @~english */
	EN_FRINGE_PATTERN_TYPE_3FREQ4STEP = 0x05,						/**< @~chinese 3频4步相移模式   @~english */
	EN_FRINGE_PATTERN_TYPE_3FREQ4STEP_MULTI_DEPTH = 0x06,			/**< @~chinese 3频4步相移+高频出2帧深度图，用于高速模式   @~english hight speed mode*/

}FRINGE_PATTERN_TYPE;

typedef struct MaxFrameTimeGain_tag
{
	int		iMaxFrame;	//the recommended value is 15000
	int		iMaxGain;	//the recommended value is 48
}MaxFrameTimeGain;

typedef struct StreamResolution_tag
{
	int iWidth;
	int iHeight;
}StreamResolution;

typedef struct DepthRoi_tag
{
	int left;		//[0,100],coordinates take the value 0-100,which represents the percentage of width or height.
	int top;		//[0,100]
	int right;		//[0,100]
	int bottom;		//[0,100]
}DepthRoi;

typedef struct RgbRoi_tag
{
	int left;		//[0,100],coordinates take the value 0-100,which represents the percentage of width or height.
	int top;		//[0,100]
	int right;		//[0,100]
	int bottom;		//[0,100]
}RgbRoi;

typedef struct HidHeader_tag
{
	int		magic;
	int		type;
	char	checksum[32]; // 0 not check, other check
	int		iDataLen;//指示acData中的有效数据大小
}HidHeader;

typedef struct WriteReadHidData_tag
{
	int iDataMax;		//指示acData最大可用空间
	int iTimeoutMs;		//指示读取数据的超时时间，单位毫秒

	unsigned char	reportId;//特别+1(ReportId)
	HidHeader		hidHeader;
	char	acData[1];		//指示具体数据

}WriteReadHidData;

//透视变换矩阵行数,列数
#define PER_TRANSF_MATRIX_ROW	4
#define PER_TRANSF_MATRIX_COL	4
#define PER_TRANSF_MATRIX_SIZE	16

/**
* @~chinese
* @brief 透视变换矩阵
* @~english
* @brief Perspective transformation matrix
**/
typedef struct PerspectiveTransformationMatrix 
{
	short width;
	short height;
	union 
	{
		float matrix[PER_TRANSF_MATRIX_ROW][PER_TRANSF_MATRIX_COL];
		float mat_[PER_TRANSF_MATRIX_SIZE];
	};
	
} PerspectiveTransformationMatrix;

/**
* @~chinese
* @brief 值范围
* @~english
* @brief Value range
**/
typedef struct ValueRange {
	float fMin_;
	float fMax_;
	float fStep_;
}*ValueRange_PTR;

/**
* @~chinese
* @brief NTC温度
* @~english
* @brief NTC temperature
**/
typedef struct NTCTempratrue {
	int ntcCnt;			//NTC个数
	int ntcTemp[10];	//NTC温度
}*NTCTempratrue_PTR;

/**
* @~chinese
* @brief Sensor温度
* @~english
* @brief Sensor temperature
**/
typedef struct SensorTempratrue {
	float sensorCnt;			//Sensor个数
	float sensorTemp[2];		//Sensor温度
}*SensorTempratrue_PTR;

/**
 * @~chinese
 * @brief	LED激光器亮度范围
 * @~english
 * @brief	luminance range of LED and laser
 */
typedef struct LUMINANCE_RANGE_LED_LASER {
	LED_ID ledId;						//LED激光机类型
	ValueRange valueRange;				//亮度值范围
}*pLUMINANCE_RANGE_LED_LASER;

/**
 * @~chinese
 * @brief	传输模式信息
 * @~english
 * @brief	communication information
 */
typedef struct COMM_MODE_INFO {
	bool isSatisfyComm;					//是否满足传输
}*pCOMM_MODE_INFO;

/**
* @~chinese
* @brief	LED灯点亮时机
* @~english
* @brief	LED illumine moment
*/
typedef enum {
	AHEAD_ILLUMINE,				//提前点亮
	DELAY_ILLUMINE,				//滞后点亮
}LEDIllumine;

/**
* @~chinese
* @brief LED点亮时机参数
* @~english
* @brief LED illumine moment
**/
typedef struct IllumineMoment {
	LEDIllumine illumineMoment;		//点亮时机,表示提前或滞后
	int advanceOrLagTime;			//提前或滞后的时间长度,单位毫秒
}*IllumineMoment_ptr;

/**
* @~chinese
* @brief LED控制参数
* @~english
* @brief LED Contrl param
**/
typedef struct LedCtrlParam {
	LED_ID emLedId;
	LED_CTRL_TYPE emCtrlType;
	unsigned int luminance;
	unsigned int timesValue;
	IllumineMoment illumineMomentParam;
}*LedCtrlParam_ptr;

/**
* @~chinese
* @brief	线激光类型
* @~english
* @brief	Laser line type
*/
typedef enum {
	LASER_LINE_CROSS,				//交叉线
	LASER_LINE_PARALLEL,			//平行线
	LASER_LINE_SINGLE,				//单线
}LaserLineType;

/**
* @~chinese
* @brief 参数设置集合
* @~english
* @brief Laser line param setting collection
**/
typedef struct ParamCollection {
	int frameTime;		//帧时间
	int exposure;		//曝光时间
	int gain;			//增益
	int laserLum;		//激光器亮
	int laserExpTimes;	//激光器曝光时长
	int irLedLum;		//IR补光灯亮度
	int irLedExpTimes;	//IR补光灯曝光时长
}*ParamCollection_ptr;

/**
* @~chinese
* @brief 激光线亮度控制参数
* @~english
* @brief LED Contrl param
**/
typedef struct LaserLineLumParam {
	LaserLineType lineType;
	unsigned int luminance;
}*LaserLineLumParam_ptr;

/**
* @~chinese
* @brief 激光线曝光时长调节参数
* @~english
* @brief Laser line exposure adjust param
**/
typedef struct LaserLineExpAdjustParam {
	LaserLineType lineType;
	unsigned int expTimes;	//单位us
}*LaserLineExpAdjustParam_ptr;

//最大支持标记点半径个数
#define SUPPORT_RADIUS_CNT_MAX	10

/**
* @~chinese
* @brief 支持标记点半径列表参数
* @~english
* @brief support radius of markers list param
**/
typedef struct SupportRadiusMarkers {
	int supportCnt;									//支持个数
	double radiusMarkers[SUPPORT_RADIUS_CNT_MAX];	//标记点半径数据
}*pSupportRadiusMarkers;

/**
* @~chinese
* @brief 扩展属性其它参数
* @~english
* @brief Ext other param
**/
typedef struct ExtOtherParam {
	char* pchParamInData;
	char* pchParamOutData;
}*ExtOtherParam_ptr;

#pragma pack(pop)

typedef enum CalibrationStreamParma
{
	DEP_STREAM = 0,			/**< @~chinese 恢复深度流			@~english Depth stream*/
	CALIBRATION_STREAM,		/**< @~chinese 标定流				@~english Calibration stream*/
	PRCZ_STREAM,			/**< @~chinese 压缩标定流			@~english Calibration stream*/
	Z16Y8Y8P_STREAM,		/**< @~chinese Z16Y8Y8P流			@~english z16y8y8p stream*/
}CalibrationStreamParma;

/**
* @~chinese
* @brief 近远景相机状态
* @~english
* @brief far near camera state
**/
typedef enum FAR_NEAR_STATE
{
	FAR_CAMERA = 0,				/**< @~chinese 远景相机		@~english far camera*/
	NEAR_CAMERA = 1,			/**< @~chinese 近景相机     @~english near camera*/
	PROCESSING = 2,				/**< @~chinese 近远景切换中 @~english processing of change btween near and far camera*/
}FAR_NEAR_STATE;

/**
* @~chinese
* @brief 近远景相机切换参数
* @~english
* @brief far near camera change param
**/
typedef struct FarNearCameraChangeParam
{
	FAR_NEAR_STATE farNearState;	/**< @~chinese 近远景相机状态		@~english near far camera state*/
	bool isAutoRun;					/**< @~chinese 是否切换后自动启流	@~english auto run flag of near far camera state change*/
}FarNearCameraChangeParam;

/**
* @~chinese
* @brief 工业相机挡板检测结果
* @~english
* @brief state of baffle from camera
**/
typedef enum BAFFLE_DETECT_RESULT
{
	RSP_SUCCESS = 0,				/**< @~chinese 成功				@~english successful*/
	NOT_START_DETECT = 1,			/**< @~chinese 未开启检测		@~english not detect yet*/
	GET_FAILED = 2,					/**< @~chinese 获取失败			@~english get failed*/
}BAFFLE_DETECT_RESULT;

/**
* @~chinese
* @brief 工业相机挡板状态
* @~english
* @brief state of baffle from camera
**/
typedef enum BAFFLE_STATE
{
	NOT_COVER = 0,				/**< @~chinese 未遮挡		@~english far camera*/
	COVERING = 1,				/**< @~chinese 已遮挡		@~english near camera*/
}BAFFLE_STATE;

/**
* @~chinese
* @brief 工业相机挡板结果
* @~english
* @brief far near camera change param
**/
typedef struct BAFFLE_DETECT_RSLT
{
	unsigned int timeout;			/**< @~chinese 获取超时值,毫秒	@~english timeout value, ms*/
	BAFFLE_DETECT_RESULT result;	/**< @~chinese 挡板检测结果		@~english detect result of baffle detect*/
	BAFFLE_STATE baffleState;		/**< @~chinese 挡板检测状态		@~english detect state of baffle*/
}BAFFLE_DETECT_RSLT;

/**
* @~chinese
* @brief 地区代码列表
* @~english
* @brief List of area code
**/
typedef struct AreaCodeList
{
	char* pAreaCodeData;				/**< @~chinese 地区代码数据,需外部初始化内存		@~english data of area code*/
	unsigned int areaCodeDataLength;	/**< @~chinese 地区代码数据长度,传入时为初始化内存大小		@~english length of data of area code*/
}*AreaCodeList_ptr;

/**
* @~chinese
* @brief 焦距倍数范围
* @~english
* @brief range of foci
**/
typedef struct FociRange
{
	int fociRange;						/**< @~chinese 焦距挡位		@~english range of foci*/
	float fociValue[56];				/**< @~chinese 焦距值		@~english max of foci*/
}*FociRange_ptr;

#ifndef DOUBLE_3
#define DOUBLE_3
struct double3 {
    double x, y, z;
    double3() { x = 0;  y = 0; z = 0; };
    double3(double a, double b, double c){x = a; y=b; z=c;}
};
#endif

#ifndef MARKER_DATA
#define MARKER_DATA
struct markerData {
    double3 point;
    double3 normal;
    double confidence;
	double radius;
	double epiolarError;
};
#endif

/**
* @~chinese
* @brief 点云数据
* @~english
* @brief point data
**/
#ifndef POINT_DATA
#define POINT_DATA
struct pointData {
	double3 point;						/**< @~chinese 点云坐标		@~english coordinates of markers */
};
#endif

/**
* @~chinese
* @brief 激光线扫标记点及点支计算标识
* @~english
* @brief state of baffle from camera
**/
typedef enum LASER_CAL_PC_MARKERS_FLAG
{
	POINT_CLOUDS = 1,				/**< @~chinese 获取点云			@~english get point clouds*/
	MARKERS = 2,					/**< @~chinese 获取标记点		@~english get markers*/
	POINTS_AND_MARKERS = 3,			/**< @~chinese 获取标记点及点云	@~english get point clouds and markers*/
	NOT_CAL = 4,					/**< @~chinese 不做任何计算		@~english not cal point clouds and markers*/				
}LASER_CAL_PC_MARKERS_FLAG;

/**
* @~chinese
* @brief 启流初始化参数
* @~english
* @brief start stream init param
**/
typedef struct START_STREAM_INIT_PARAM
{
	bool isSet;					/**< @~chinese 是否设置,us				@~english flag of set*/
	float frameTime;			/**< @~chinese 帧时间,us				@~english frame time, us*/
	float expTime;				/**< @~chinese 曝光时间,us				@~english expourse time, us*/
	float gain;					/**< @~chinese 增益,us					@~english gain*/
	int luminanceIrLed;			/**< @~chinese IR补光灯亮度				@~english luminance of ir led*/
	int luminanceLaser;			/**< @~chinese 光机亮度					@~english luminance of projector*/
	bool isSetRetrun;			/**< @~chinese 是否设置回程扫描开关		@~english is set swith of return scan*/
	bool returnScanSwith;		/**< @~chinese 回程扫描开关				@~english swith of return scan*/
}*START_STREAM_INIT_PARAM_ptr;

/**
* @~chinese
* @brief 设备(相机)的指标灯状态，只适用于部分相机
* @~english
* @brief indicator light status of device(camera)
**/
typedef enum INDICATOR_LIGHT_MODE
{
	LIGHTING_OFF = 0,						/**< @~chinese 灯熄灭状态			@~english lighting off state*/
	RED_LIGHT_CONSTANTLY_LIT = 1,			/**< @~chinese 红灯常亮状态			@~english Red light constantly lit*/
	RED_LIGHT_FLASHING = 2,					/**< @~chinese 红灯闪亮状态			@~english Red light flashing*/
	GREEN_LIGHT_CONSTANTLY_LIT = 3,			/**< @~chinese 绿灯常亮状态			@~english Green light constantly lit*/
}INDICATOR_LIGHT_MODE;

/**
* @~chinese
* @brief 线激光点云重建计算方式
* @~english
* @brief Calculation Method for Line Laser Point Cloud Reconstruction
**/
typedef enum LASER_LINE_PC_CAL_MODE
{
	PC_CAL_MODE_CPU = 0,						/**< @~chinese CPU计算			@~english CPU calculation*/
	PC_CAL_MODE_GPU = 1,						/**< @~chinese GPU计算			@~english GPU calculation*/
}LASER_LINE_PC_CAL_MODE;

/**
* @~chinese
* @brief 线激光点云重建计算方式参数
* @~english
* @brief Calculation Method param for Line Laser Point Cloud Reconstruction
**/
typedef struct LASER_LINE_PC_CAL_PARAM
{
	LASER_LINE_PC_CAL_MODE calMode;					/**< @~chinese 计算方式				@~english calculation mode*/
	char gpuName[500];								/**< @~chinese GPU名称				@~english GPU name*/
	int gpuId;										/**< @~chinese GPU序号				@~english GPU id*/
	int queueSize;									/**< @~chinese 缓冲队列长度			@~english Buffer Queue Length*/
}*LASER_LINE_PC_CAL_PARAM_ptr;

/**
* @~chinese
* @brief 相机启动状态
* @~english
* @brief state of camera
**/
typedef enum CAMERA_BOOT_STATE
{
	NORMAL = 0,						/**< @~chinese 正常状态			@~english normal state*/
	FPGA_UPGRADE = 1,				/**< @~chinese GPGA升级中		@~english upgrade of FPGA*/
}CAMERA_BOOT_STATE;

/// \~chinese
/// \defgroup PropertyExtensionType 扩展属性
/// \brief 列举所有可设置的扩展属性
/// @{
/// \~english
/// \defgroup PropertyExtensionType Extensional property
/// \brief List extensional properties
/// @{

/**
* @~chinese
* @brief 枚举: 扩展属性
* @~english
* @brief enumeration: extensional of property
**/
typedef enum PROPERTY_TYPE_EXTENSION
{
	PROPERTY_EXT_DEPTH_SCALE				= 0x0,		/**< @~chinese 深度值缩放系数               @~english depth unit for real distance */
	PROPERTY_EXT_TRIGGER_MODE				= 0x1,		/**< @~chinese 触发模式                     @~english/PROPERTY_EXT_TRIGGER_OUT_MODE set trigger mode ,normal or trigge mode, value 1 stands for software trigger mode, value 2 stands for hardware trigger mode, other stands for trigger off(default)*/
	PROPERTY_EXT_TRIGGER_OUT_MODE			= 0x2,		/**< @~chinese 是否开启脉冲输出             @~english enable/disable trigger out*/
	PROPERTY_EXT_HDR_BRIGHTNESS				= 0x3,		/**< @~chinese HDR各级参数                  @~english all params of HDR*/
	PROPERTY_EXT_IP_SETTING					= 0x4,		/**< @~chinese Client模式的IP设置           @~english Client Mode IP setting*/
	PROPERTY_EXT_NETWORK_COMPRESS			= 0x5,		/**< @~chinese 网络传输时流是否压缩			@~english whether the stream compresses when transmited by network*/
	PROPERTY_EXT_SERIAL_NUMBER				= 0x6,		/**< @~chinese 修改序列号*/
	PROPERTY_EXT_MAC_ADDRESS				= 0x7,		/**< @~chinese 修改mac地址*/
	PROPERTY_EXT_CAMERA_IP					= 0x8,		/**< @~chinese IP设置                       @~english IP setting*/
	PROPERTY_EXT_AP_NET_WORKING_INFO		= 0x9,		/**< @~chinese ap模式联网 属性				@~english property of net working,reference NetworkingInfo*/
	PROPERTY_EXT_FAST_SCAN_MODE				= 0x0A,		/**< @~chinese 快速扫描模式					@~english property of fast scan mode ,reference bFastScanMode*/
	PROPERTY_EXT_HARDSYNCSWITCH				= 0x0B,		/**< @~chinese 硬同步开关					@~english property of hard synchronization ,reference bHardSyncSwitch*/
	PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM		= 0x0C,		/**< @~chinese 深度和RGB帧匹配参数			@~english property of depth and rgb match param ,reference DepthRgbMatchParam*/
	PROPERTY_EXT_PAUSE_DEPTH_STREAM			= 0x0D,		/**< @~chinese 暂停深度流					@~english pause the depth stream*/
	PROPERTY_EXT_RESUME_DEPTH_STREAM		= 0x0E,		/**< @~chinese 恢复深度流					@~english resume the depth stream*/
	PROPERTY_EXT_TERMINAL_NET_WORKING_INFO	= 0x10,		/**< @~chinese 终端模式联网 属性			@~english property of net working,reference NetworkingInfo*/
	PROPERTY_EXT_IS_WIFI_HOST_MODE			= 0x11,		/**< @~chinese 判断是否在wifi-host模式 属性	@~english property of bool ,which is in wifi host mode,reference bIsWifiHostMode*/
	PROPERTY_EXT_CLEAR_FRAME_BUFFER			= 0x13,		/**< @~chinese 清除SDK内部的Frame队列		@~english empty sdk frame buffer*/
	PROPERTY_EXT_GET_RECONSTRUCTIONMAT		= 0x14,		/**< @~chinese 获取视差重建点云的矩阵		@~english get parallax reconstruction point cloud.*/
	PROPERTY_EXT_EXPOSURE_TIME_RGB			= 0x15,		/**< @~chinese 曝光时间,单位微秒,仅限RGB	@~english exposure time,unit us.*/
	PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB	= 0x16,		/**< @~chinese 曝光时间范围,仅限RGB			@~english exposure time,unit us.*/
	PROPERTY_EXT_CPU_TEMPERATURE			= 0x17,		/**< @~chinese CPU温度,单位摄氏度,			@~english CPU temperature,unit degree Celsius */
	PROPERTY_EXT_NTC_TEMPERATURE			= 0x18,		/**< @~chinese NTC温度,单位摄氏度,有多个	@~english NTC temperature,unit degree Celsius */
	PROPERTY_EXT_SENSOR_TEMPERATURE			= 0x19,		/**< @~chinese Sensor温度,单位摄氏度		@~english Sensor temperature,unit degree Celsius */
	PROPERTY_EXT_IS_SUPPORT_GYRO			= 0x20,		/**< @~chinese 判断相机是否支持陀螺仪		@~english the camera supports gyroscope or not, reference bSupportGyro*/
	PROPERTY_EXT_GET_GYRO_VERSION			= 0x21,		/**< @~chinese 获取相机陀螺仪版本信息		@~english get camera gyroscope version information,reference gyroVersion */
	PROPERTY_EXT_PARAM_FILE_MNG				= 0x22,		/**< @~chinese 参数文件管理					@~english camera paramer file management*/
	PROPERTY_EXT_FAR_NEAR_CAMERA_STATE		= 0x23,		/**< @~chinese 近远景相机状态				@~english far near camera state management*/
	PROPERTY_EXT_CLOCK_SYNCHRONIZATION		= 0x24,		/**< @~chinese 主机与相机时钟同步			@~english the clock synchronization between host and camera*/
	PROPERTY_EXT_CAMERA_CLOCK_CUR			= 0x25,		/**< @~chinese 相机当前时钟					@~english the clock for the camera*/
	PROPERTY_EXT_CLOCK_DELTA				= 0x26,		/**< @~chinese 主机与相机时钟差值			@~english the clock delta between host and camera*/

	PROPERTY_EXT_SET_FAKE_MODE				= 0x904,	/**< @~chinese 设置真假分辨率				@~english set fake mode */
	PROPERTY_EXT_AUTO_EXPOSURE_MODE			= 0x912,	/**< @~chinese 深度相机自动曝光模式			@~english auto exposure mode of depth camera，reference AUTO_EXPOSURE_MODE*/
	PROPERTY_EXT_DEPTH_ROI					= 0x913,	/**< @~chinese 深度数据的ROI				@~english roi of depth data,reference DepthRoi*/
	PROPERTY_EXT_HDR_MODE					= 0x914,	/**< @~chinese HDR模式						@~english HDR mode*/
	PROPERTY_EXT_HDR_SCALE_SETTING			= 0x915,	/**< @~chinese HDR自动模式的配置			@~english setting of auto-HDR*/
	PROPERTY_EXT_HDR_EXPOSURE				= 0x916,	/**< @~chinese HDR各级曝光参数				@~english all exposure params of HDR*/
	PRPOERTY_EXT_MAX_FRAMETIME_GAIN			= 0x917,	/**< @~chinese 最大帧时间和增益	            @~english set the max frame and gran,reference MaxFrameTimeGain*/
	PROPERTY_EXT_SPEED						= 0x924,	/**< @~chinese 拍图速度						@~english speed of shot*/
	PROPERTY_EXT_RGB_ROI					= 0x925,	/**< @~chinese RGB数据的ROI					@~english roi of RGB data,reference RgbRoi*/
	PROPERTY_EXT_IS_SUPPORT_RGB_ROI			= 0x926,	/**< @~chinese RGB数据的ROI					@~english roi of RGB data,reference RgbRoi*/

	PROPERTY_EXT_SET_STREAM_RESOLUTION		= 0x702,	/**< @~chinese 设置分辨率					@~english set resolution of depth stream，reference StreamResolution*/
	PROPERTY_EXT_CONTRAST_MIN				= 0x705,	/**< @~chinese 对比度阈值，此阈值用于删除原始图像中低曝光区域,将删除灰度低于该阈值的区域,阈值越大低曝光区域删除越多.调节范围0~40灰阶,建议设置为5
																									@~english remove where fringe contrast below this value*/
	PROPERTY_EXT_ALGO_SET_BACKGROUND		= 0x705,	/**< @~chinese 背景阈值,范围[0-40]			@~english background threshold,[0,40]，reference algoSetBackground*/
	PROPERTY_EXT_ALGO_SET_GRADIENT			= 0x706,	/**< @~chinese 梯度阈值,范围[0-1000]		@~english gradient threshold,[0,40]，reference algoSetGradient*/
	PROPERTY_EXT_DEPTH_RANGE				= 0x707,	/**< @~chinese 深度范围						@~english depth range of camera*/
	PROPERTY_EXT_CHANGE_CAL_STREA_FMT		= 0x708,	/**< @~chinese 标定切换流格式				@~english change stream format*/
	PROPERTY_EXT_FILTERING_SWITCH			= 0x70a,	/**< @~chinese 设置滤波开关值				@~english set the filter switch value*/

	PROPERTY_EXT_UNKNOWA00					= 0xa00,	/**< @~chinese								@~english */
	PROPERTY_EXT_MULTIFRAME_FUSION			= 0xa04,	/**< @~chinese 多帧融合(双曝光)，取值0/1	@~english multiframe fusion,reference multiframeFusion*/
	PROPERTY_EXT_SET_FRINGE_PATTERN			= 0xa05,	/**< @~chinese 条纹pattern参数				@~english fringe pattern,reference FRINGE_PATTERN_TYPE*/
	PROPERTY_EXT_LED_ON_OFF					= 0xb00,	/**< @~chinese 是否打开LED灯				@~english turn on/off led*/
	PROPERTY_EXT_LED_CTRL					= 0xb01,	/**< @~chinese LED灯控制					@~english led contrl*/
	PROPERTY_EXT_BAFFLE_DETECT_STATE		= 0xb02,	/**< @~chinese 工业相机挡板检测状态			@~english baffle detect state*/
	PROPERTY_EXT_AREA_CODE					= 0xb03,	/**< @~chinese 地区代码						@~english Area code*/
	PROPERTY_EXT_AREA_CODE_LIST				= 0xb04,	/**< @~chinese 地区代码列表					@~english Area code list*/
	PROPERTY_EXT_BAFFLE_DETECT_RESULT		= 0xb05,	/**< @~chinese 工业相机挡板检测结果			@~english baffle detect result*/
	PROPERTY_EXT_RETURN_SCAN				= 0xb06,	/**< @~chinese 回程扫描						@~english return scan*/
	PROPERTY_EXT_LED_LASER_CTRL_SUPPORT		= 0xb07,	/**< @~chinese 是否支持LED激光控制			@~english led contrl*/
	PROPERTY_EXT_IS_SUPPORT_FOCI_MULTIPLE	= 0xb08,	/**< @~chinese 是否支持焦距倍数控制			@~english is support foci multiple contrl*/
	PROPERTY_EXT_FOCI_MULTIPLE_RANGE		= 0xb09,	/**< @~chinese 焦距倍数范围					@~english foci multiple range*/
	PROPERTY_EXT_FOCI_MULTIPLE				= 0xb0a,	/**< @~chinese 焦距倍数控制					@~english foci multiple contrl*/
	PROPERTY_EXT_PROJECTOR_SWITCH_IN_WP		= 0xb0b,	/**< @~chinese 白条纹模式光机控制			@~english projector switch contrl in white pattern mode*/
	PROPERTY_EXT_START_STAREAM_INIT_PARAM	= 0xb0c,	/**< @~chinese 启流初始化参数				@~english start stream init param*/
	PROPERTY_EXT_LASER_SCAN_FORMAT_CAHANGE	= 0xb0d,	/**< @~chinese 线扫格式切换					@~english laser scan format change*/
	PROPERTY_EXT_ENTER_LASER_GROUP			= 0xb0e,	/**< @~chinese 激光线集合IR流				@~english laser line group mode in IR stream*/
	PROPERTY_EXT_LASER_CAL_PC_MARKERS_FLAGE	= 0xb0f,	/**< @~chinese 激光线扫标记点点云计算标识	@~english cal points and markers cal flag of laser scan*/
	PROPERTY_EXT_LASER_LED_LUMINANCE_RANGE	= 0xb10,	/**< @~chinese 激光器LED亮度值范围			@~english the luminance range of laser and led*/
	PROPERTY_EXT_PROJECTOR_WP_IS_SUPPORT	= 0xb11,	/**< @~chinese 是否支持白条纹模式光机控制	@~english is support projector switch contrl in white pattern mode*/
	PROPERTY_EXT_GET_COMMUNICATION_INFO		= 0xb12,	/**< @~chinese 获取USB传输信息				@~english get communication information*/
	PROPERTY_EXT_INDICATOR_LIGHT_MODE		= 0xb13,	/**< @~chinese 设备指示灯状态				@~english indicator light mode of camera device*/
	PROPERTY_EXT_LASER_CAL_MODEL			= 0xb14,	/**< @~chinese 激光线点云计算方式			@~english mode of laser point cloud reconstuct*/
	PROPERTY_EXT_CAMERA_BOOT_STATE			= 0xb15,	/**< @~chinese 相机启动状态					@~english state of camera internal*/
	PROPERTY_EXT_LASER_LINE_LUM_CTRL		= 0xb16,	/**< @~chinese 激光线亮度控制				@~english laser line lumience ctrl*/
	PROPERTY_EXT_RED_LIGHT_INDICATOR		= 0xb17,	/**< @~chinese 红线指示灯状态				@~english red light indicator*/
	PROPERTY_EXT_GET_NETWORK_SPEED			= 0xb18,	/**< @~chinese 网络带宽						@~english network speed*/
	PROPERTY_EXT_MARKER_SUPPORT_RADIUS		= 0xb19,	/**< @~chinese 标记点支持半径				@~english support radius of markers*/
	PROPERTY_EXT_IS_SUPPORT_DEP_AUTO_EXP	= 0xb20,	/**< @~chinese 是否支持深度自动曝光			@~english is support depth auto exposure*/
	PROPERTY_EXT_LASER_LINE_EXP_TIME_ADJUST = 0xb21,	/**< @~chinese 激光线曝光时长控制			@~english laser line laser line exposure time adjust*/
	PROPERTY_EXT_LASER_DIRECTION_CAHANGE	= 0xb22,	/**< @~chinese 激光线方向切换,不重建点云	@~english laser direction change*/
	PROPERTY_EXT_LASER_PARAM_COLLECTION		= 0xb23,	/**< @~chinese 激光线参数设置集合			@~english laser setting param collection*/

	PROPERTY_EXT_TRIGGER_IN_MODE			= 0x106,	/**< @~chinese 触发模式                     @~english trigger mode*/
	PROPERTY_EXT_OTHER_PARAM				= 0xeeee,	/**< @~chinese								@~english */

	//不对外属性
	PROPERTY_EXT_EXECUTE_CMD				= 0xffff,	/**< @~chinese 执行任意命令*/
	PROPERTY_EXT_SYS_CMD					= 0xaaaa,	/**< @~chinese 按规定格式执行命令,具体参考用例*/
	PROPERTY_EXT_WRITE_READ_HID				= 0x1111,	/**< @~chinese 写并等待读取hid数据*/
} PROPERTY_TYPE_EXTENSION;
/// @}

/**
* @~chinese
* @brief 扩展属性值，联合体表示，设置和获取时只取指定属性对应的字段即可
* @~english
* @brief union of extensional property
**/
typedef union PropertyExtension
{
	float depthScale;							/**< @~chinese 对应PROPERTY_EXT_DEPTH_SCALE			    @~english corresponding PROPERTY_EXT_DEPTH_SCALE			*/
	TRIGGER_MODE triggerMode;					/**< @~chinese 对应PROPERTY_EXT_TRIGGER_MODE			@~english corresponding PROPERTY_EXT_TRIGGER_MODE			*/
	int algorithmContrast;						/**< @~chinese 对应PROPERTY_EXT_CONTRAST_MIN			@~english corresponding PROPERTY_EXT_CONTRAST_MIN			*/
	AUTO_EXPOSURE_MODE autoExposureMode;		/**< @~chinese 对应PROPERTY_EXT_AUTO_EXPOSURE_MODE	    @~english corresponding PROPERTY_EXT_AUTO_EXPOSURE_MODE	*/
	HdrScaleSetting hdrScaleSetting;			/**< @~chinese 对应PROPERTY_EXT_HDR_SCALE_SETTING	    @~english corresponding PROPERTY_EXT_HDR_SCALE_SETTING	*/
	HdrExposureSetting hdrExposureSetting;		/**< @~chinese 对应PROPERTY_EXT_HDR_EXPOSURE			@~english corresponding PROPERTY_EXT_HDR_EXPOSURE			*/
	int ledOnOff;								/**< @~chinese 对应PROPERTY_EXT_LED_ON_OFF			@~english corresponding PROPERTY_EXT_LED_ON_OFF			*/
	
	TRIGGER_MODE triggerInMode;					/**< @~chinese 对应PROPERTY_EXT_TRIGGER_OUT_MODE		@~english corresponding PROPERTY_EXT_TRIGGER_OUT_MODE	 */
	int triggerOutEnable;						/**< @~chinese 对应PROPERTY_EXT_TRIGGER_IN_MODE		    @~english corresponding PROPERTY_EXT_TRIGGER_IN_MODE	 */
	int laserBrightness;						/**< @~chinese 对应PROPERTY_EXT_LASER_BRIGHTNESS		@~english corresponding PROPERTY_EXT_LASER_BRIGHTNESS	 */
	int laserOnOff;								/**< @~chinese 对应PROPERTY_EXT_LASER_ON_OFF			@~english corresponding PROPERTY_EXT_LASER_ON_OFF		 */
	int speed;									/**< @~chinese 对应PROPERTY_EXT_SPEED				    @~english corresponding PROPERTY_EXT_SPEED				 */
	HdrBrightnessSetting hdrBrightnessSetting;	/**< @~chinese 对应PROPERTY_EXT_HDR_BRIGHTNESS		    @~english corresponding PROPERTY_EXT_HDR_BRIGHTNESS		 */

	HDR_MODE hdrMode;							/**< @~chinese 对应PROPERTY_EXT_HDR_MODE				@~english corresponding PROPERTY_EXT_HDR_MODE				*/
	DepthRange depthRange;						/**< @~chinese 对应PROPERTY_EXT_DEPTH_RANGE			    @~english corresponding PROPERTY_EXT_DEPTH_RANGE			*/
	IpSetting ipSetting;						/**< @~chinese deprecated 对应PROPERTY_EXT_IP_SETTING	@~english deprecated corresponding PROPERTY_EXT_IP_SETTING*/
    CameraIpSetting cameraIp;                   /**< @~chinese 对应PROPERTY_EXT_CAMERA_IP		    @~english corresponding PROPERTY_EXT_CAMERA_IP*/
    NETWORK_COMPRESS_MODE networkCompressMode;	/**< @~chinese 对应PROPERTY_EXT_NETWORK_COMPRESS		@~english corresponding PROPERTY_EXT_NETWORK_COMPRESS	*/
	NetworkingInfo		networkingInfo;			/**< @~chinese 对应PROPERTY_EXT_AP_NET_WORKING_INFO/PROPERTY_EXT_TERMINAL_NET_WORKING_INFO		@~english corresponding PROPERTY_EXT_AP_NET_WORKING_INFO/PROPERTY_EXT_TERMINAL_NET_WORKING_INFO	*/
	bool				bFastScanMode;			/**< @~chinese 对应PROPERTY_EXT_FAST_SCAN_MODE			@~english corresponding PROPERTY_EXT_FAST_SCAN_MODE	*/
	bool				bHardSyncSwitch;		/**< @~chinese 对应PROPERTY_EXT_HARDSYNCSWITCH			@~english corresponding PROPERTY_EXT_HARDSYNCSWITCH	*/
	DepthRgbMatchParam	depthRgbMatchParam;		/**< @~chinese 对应PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM	@~english corresponding PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM	*/
	FRINGE_PATTERN_TYPE	fringePatternType;		/**< @~chinese 对应PROPERTY_EXT_SET_FRINGE_PATTERN	@~english corresponding PROPERTY_EXT_SET_FRINGE_PATTERN	*/
	int					algoSetBackground;		/**< @~chinese 对应PROPERTY_EXT_ALGO_SET_BACKGROUND	@~english corresponding PROPERTY_EXT_ALGO_SET_BACKGROUND	*/
	int					algoSetGradient;		/**< @~chinese 对应PROPERTY_EXT_ALGO_SET_GRADIENT	@~english corresponding PROPERTY_EXT_ALGO_SET_GRADIENT	*/
	bool				multiframeFusion;		/**< @~chinese 对应PROPERTY_EXT_MULTIFRAME_FUSION	@~english corresponding PROPERTY_EXT_MULTIFRAME_FUSION	*/
	bool				bIsWifiHostMode;		/**< @~chinese 对应PROPERTY_EXT_IS_WIFI_HOST_MODE	@~english corresponding PROPERTY_EXT_IS_WIFI_HOST_MODE	*/
	STREAM_TYPE			streamType;				/**< @~chinese 对应PROPERTY_EXT_CLEAR_FRAME_BUFFER	@~english corresponding PROPERTY_EXT_CLEAR_FRAME_BUFFER	*/
	MaxFrameTimeGain	maxFrameTimeGain;		/**< @~chinese 对应PRPOERTY_EXT_MAX_FRAMETIME_GAIN	@~english corresponding PRPOERTY_EXT_MAX_FRAMETIME_GAIN	*/
	StreamResolution	streamResolution;		/**< @~chinese 对应PROPERTY_EXT_SET_STREAM_RESOLUTION	@~english corresponding PROPERTY_EXT_SET_STREAM_RESOLUTION	*/
	char acSerialNumber[60];					/**< @~chinese 对应PROPERTY_EXT_SERIAL_NUMBER	@~english corresponding PROPERTY_EXT_SERIAL_NUMBER	*/
	char acMac[60];								/**< @~chinese 对应PROPERTY_EXT_MAC_ADDRESS	@~english corresponding PROPERTY_EXT_MAC_ADDRESS	*/
	DepthRoi	depthRoi;						/**< @~chinese 对应PROPERTY_EXT_DEPTH_ROI	@~english corresponding PROPERTY_EXT_DEPTH_ROI	*/
	WriteReadHidData*	writeReadHidData;		/**< @~chinese 对应PROPERTY_EXT_WRITE_READ_HID	@~english corresponding PROPERTY_EXT_WRITE_READ_HID	*/
	PerspectiveTransformationMatrix	reconstructionMat; /**< @~chinese 对应PROPERTY_EXT_GET_RECONSTRUCTIONMAT	@~english corresponding PROPERTY_EXT_GET_RECONSTRUCTIONMAT	*/
	RgbRoi	rgbRoi;								/**< @~chinese 对应PROPERTY_EXT_RGB_ROI	@~english corresponding PROPERTY_EXT_RGB_ROI	*/

	unsigned int  uiExposureTime;	            /**< @~chinese 对应PROPERTY_EXT_EXPOSURE_TIME_RGB 曝光时间,单位毫秒  @~english corresponding PROPERTY_EXT_EXPOSURE_TIME exposure time,unit us*/
	ValueRange objVRange_;						/**< @~chinese 对应PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB 曝光时间范围,单位毫秒  @~english corresponding PROPERTY_EXT_EXPOSURE_TIME_RANGE exposure time range,unit us*/
	unsigned int cpuTempratrue;					/**< @~chinese 对应PROPERTY_EXT_CPU_TEMPRATRUE  @~english corresponding PROPERTY_EXT_CPU_TEMPRATRUE exposure temperature,unit degree Celsius*/
	NTCTempratrue ntcTempratrue;				/**< @~chinese 对应PROPERTY_EXT_NTC_TEMPRATRUE  @~english corresponding PROPERTY_EXT_NTC_TEMPRATRUE exposure temperature,unit degree Celsius*/
	SensorTempratrue sensorTempratrue;			/**< @~chinese 对应PROPERTY_EXT_SENSOR_TEMPERATURE  @~english corresponding PROPERTY_EXT_SENSOR_TEMPERATURE temperature,unit degree Celsius*/

	bool boolCameraState;						/**< @~chinese 对应PROPERTY_EXT_GET_CAMERA_STATE 相机当前连接状态  @~english Camera current connect state*/

	bool bSupportGyro;                          /**< @~chinese 对应PROPERTY_EXT_IS_SUPPORT_GYRO   @~english corresponding PROPERTY_EXT_IS_SUPPORT_GYRO */
	unsigned short gyroVersion;                 /**< @~chinese 对应PROPERTY_EXT_GET_GYRO_VERSION  @~english corresponding PROPERTY_EXT_GET_GYRO_VERSION */
	CameraParamMng cameraParamMng;				/**< @~chinese 对应PROPERTY_EXT_PARAM_FILE_MNG  @~english corresponding PROPERTY_EXT_PARAM_FILE_MNG */
	CalibrationStreamParma calibrationStreamParam; /**< @~chinese 对应PROPERTY_EXT_CHANGE_CAL_STREA_FMT  @~english corresponding PROPERTY_EXT_CHANGE_CAL_STREA_FMT */
	int filterSwitch;							/**< @~chinese 对应PROPERTY_EXT_FILTERING_SWITCH  @~english corresponding PROPERTY_EXT_FILTERING_SWITCH */
				 
	LedCtrlParam ledCtrlParam;					/**< @~chinese 对应PROPERTY_EXT_LED_CTRL  @~english corresponding PROPERTY_EXT_LED_CTRL */
	bool isDetectBaffle;						/**< @~chinese 对应PROPERTY_EXT_BAFFLE_DETECT_STATE  @~english corresponding PROPERTY_EXT_BAFFLE_DETECT_STATE */
	BAFFLE_DETECT_RSLT baffleDetectRslt;		/**< @~chinese 对应PROPERTY_EXT_BAFFLE_DETECT_RESULT  @~english corresponding PROPERTY_EXT_BAFFLE_DETECT_RESULT */
	bool retrunScanSwitch;						/**< @~chinese 对应PROPERTY_EXT_RETURN_SCAN  @~english corresponding PROPERTY_EXT_RETURN_SCAN */
	LED_CTRL_IS_SUPPORT ledLaserCtrlSupport;	/**< @~chinese 对应PROPERTY_EXT_LED_LASER_CTRL_SUPPORT  @~english corresponding PROPERTY_EXT_LED_LASER_CTRL_SUPPORT */
	bool isSupportFociCtrl;						/**< @~chinese 对应PROPERTY_EXT_IS_SUPPORT_FOCI_MULTIPLE  @~english corresponding PROPERTY_EXT_IS_SUPPORT_FOCI_MULTIPLE */
	FociRange fociRange;						/**< @~chinese 对应PROPERTY_EXT_FOCI_MULTIPLE_RANGE  @~english corresponding PROPERTY_EXT_FOCI_MULTIPLE_RANGE */
	float fociValue;							/**< @~chinese 对应PROPERTY_EXT_FOCI_MULTIPLE  @~english corresponding PROPERTY_EXT_FOCI_MULTIPLE */
	bool projectorCtrlInWP;						/**< @~chinese 对应PROPERTY_EXT_PROJECTOR_SWITCH_IN_WP  @~english corresponding PROPERTY_EXT_PROJECTOR_SWITCH_IN_WP */
	START_STREAM_INIT_PARAM startInitParam;		/**< @~chinese 对应PROPERTY_EXT_START_STAREAM_INIT_PARAM  @~english corresponding PROPERTY_EXT_START_STAREAM_INIT_PARAM */
	STREAM_FORMAT laserScanFormat		;		/**< @~chinese 对应PROPERTY_EXT_LASER_SCAN_FORMAT_CAHANGE  @~english corresponding PROPERTY_EXT_LASER_SCAN_FORMAT_CAHANGE */
	bool isLaserLineGroup;						/**< @~chinese 对应PROPERTY_EXT_ENTER_LASER_GROUP  @~english corresponding PROPERTY_EXT_ENTER_LASER_GROUP */
	LASER_CAL_PC_MARKERS_FLAG laserCalFlag;		/**< @~chinese 对应PROPERTY_EXT_LASER_CAL_PC_MARKERS_FLAGE  @~english corresponding PROPERTY_EXT_LASER_CAL_PC_MARKERS_FLAGE */
	LUMINANCE_RANGE_LED_LASER rangeLuminance;	/**< @~chinese 对应PROPERTY_EXT_LASER_LED_LUMINANCE_RANGE  @~english corresponding PROPERTY_EXT_LASER_LED_LUMINANCE_RANGE */
	bool isSupportWPCtrl;						/**< @~chinese 对应PROPERTY_EXT_PROJECTOR_WP_IS_SUPPORT  @~english corresponding PROPERTY_EXT_PROJECTOR_WP_IS_SUPPORT */
	COMM_MODE_INFO comModeInfo;					/**< @~chinese 对应PROPERTY_EXT_GET_COMMUNICATION_INFO  @~english corresponding PROPERTY_EXT_GET_COMMUNICATION_INFO */
	
	FarNearCameraChangeParam farNearStateParam;	/**< @~chinese 对应PROPERTY_EXT_FAR_NEAR_CAMERA_STATE  @~english corresponding PROPERTY_EXT_FAR_NEAR_CAMERA_STATE */					
	FAR_NEAR_STATE farNearState;				/**< @~chinese 对应PROPERTY_EXT_FAR_NEAR_CAMERA_STATE  @~english corresponding PROPERTY_EXT_FAR_NEAR_CAMERA_STATE */
	unsigned int syncTimes;						/**< @~chinese 对应PROPERTY_EXT_CLOCK_SYNCHRONIZATION  @~english corresponding PROPERTY_EXT_CLOCK_SYNCHRONIZATION */
	std::uint64_t clockCur;						/**< @~chinese 对应PROPERTY_EXT_CAMERA_CLOCK_CUR  @~english corresponding PROPERTY_EXT_CAMERA_CLOCK_CUR */
	std::uint64_t clockDelta;					/**< @~chinese 对应PROPERTY_EXT_CLOCK_DELTA  @~english corresponding PROPERTY_EXT_CLOCK_DELTA */
	ExtOtherParam extOtherParam;				/**< @~chinese 对应PROPERTY_EXT_OTHER_PARAM  @~english corresponding PROPERTY_EXT_OTHER_PARAM */
	char areaCodeCul[10];						/**< @~chinese 对应PROPERTY_EXT_AREA_CODE  @~english corresponding PROPERTY_EXT_AREA_CODE */
	AreaCodeList areaCodeList;					/**< @~chinese 对应PROPERTY_EXT_AREA_CODE_LIST  @~english corresponding PROPERTY_EXT_AREA_CODE_LIST */
	INDICATOR_LIGHT_MODE indicatorLightMode;	/**< @~chinese 对应PROPERTY_EXT_INDICATOR_LIGHT_MODE  @~english corresponding PROPERTY_EXT_INDICATOR_LIGHT_MODE */
	LASER_LINE_PC_CAL_PARAM laserLineCalParam;	/**< @~chinese 对应PROPERTY_EXT_LASER_CAL_MODEL  @~english corresponding PROPERTY_EXT_LASER_CAL_MODEL */
	CAMERA_BOOT_STATE stateBooting;				/**< @~chinese 对应PROPERTY_EXT_CAMERA_BOOT_STATE  @~english corresponding PROPERTY_EXT_CAMERA_BOOT_STATE */
	LaserLineLumParam laserLineLumParam;		/**< @~chinese 对应PROPERTY_EXT_LASER_LINE_LUM_CTRL  @~english corresponding PROPERTY_EXT_LASER_LINE_LUM_CTRL */
	bool redLightIndicator;						/**< @~chinese 对应PROPERTY_EXT_RED_LIGHT_INDICATOR  @~english corresponding PROPERTY_EXT_RED_LIGHT_INDICATOR */
	int networkSpeed;							/**< @~chinese 对应PROPERTY_EXT_GET_NETWORK_SPEED  @~english corresponding PROPERTY_EXT_GET_NETWORK_SPEED */
	SupportRadiusMarkers supportRadius;			/**< @~chinese 对应PROPERTY_EXT_MARKER_SUPPORT_RADIUS  @~english corresponding PROPERTY_EXT_MARKER_SUPPORT_RADIUS */
	bool isSupportDepAutoExp;					/**< @~chinese 对应PROPERTY_EXT_IS_SUPPORT_DEP_AUTO_EXP  @~english corresponding PROPERTY_EXT_IS_SUPPORT_DEP_AUTO_EXP */
	LaserLineExpAdjustParam laserLineExpTime;	/**< @~chinese 对应PROPERTY_EXT_LASER_LINE_EXP_TIME_ADJUST  @~english corresponding PROPERTY_EXT_LASER_LINE_EXP_TIME_ADJUST */
	LaserLineType lineDirType;					/**< @~chinese 对应PROPERTY_EXT_LASER_DIRECTION_CAHANGE  @~english corresponding PROPERTY_EXT_LASER_DIRECTION_CAHANGE */
	ParamCollection	paramCol;					/**< @~chinese 对应PROPERTY_EXT_LASER_PARAM_COLLECTION  @~english corresponding PROPERTY_EXT_LASER_PARAM_COLLECTION */
	
	char reservedStr[1024];                     /**< @~chinese 预留									    @~english reserved */
    int reserved[15];							/**< @~chinese 预留									    @~english reserved */
}PropertyExtension;

#define KEY_INFO_RESERVED_LEN	2

#pragma pack(push, 1)
/**
* @~chinese
* @brief 按键信息
* @~english
* @brief key information
**/
typedef struct KeyInfo {
    char   key_num;
	char   key_level;	//KEY_EVENT_DOWN = 0,KEY_EVENT_UP = 1
	char   reserved[KEY_INFO_RESERVED_LEN];	//reserved fot 2 bytes
    int   key_time;		//按键时间，特别说明：设备上电到第一次按键值一直为0
}KeyInfo;

/**
* @~chinese
* @brief	陀螺仪位置和位姿
* @~english
* @brief position and pose of gyro
*/
/*
typedef struct GyroPositionAndPose_tag
{
	float	x;
	float	y;
	float	z;
	float	roll;
	float	pitch;
	float	yaw;
}GyroPositionAndPose;
*/
typedef struct GyroPositionAndPose_tag
{	
	float q0; // w					/**< (q0,q1,q2,q3)->(w,x,y,z): @~chinese 陀螺仪姿态，使用四元素表示  @~english pose of gyro，Use four elements to represent */
	float q1; // x
	float q2; // y
	float q3; // z
	float gx;						/**< @~chinese 陀螺仪绕X轴转动角速度，单位为：radian/second   @~english  Gyroscope rotates angular velocity around the X axis, unit: radian/second */
	float gy;						/**< @~chinese  陀螺仪绕Y轴转动角速度，单位为：radian/second   @~english  Gyroscope rotates angular velocity around the Y axis, unit: radian/second */
	float gz;						/**< @~chinese  陀螺仪绕Z轴转动角速度，单位为：radian/second  @~english  Gyroscope rotates angular velocity around the Z axis, unit: radian/second */
	float ax;						/**< @~chinese  陀螺仪X方向加速度，单位为：g  @~english  Acceleration in the X direction of the gyroscope, unit: g */
	float ay;						/**< @~chinese  陀螺仪Y方向加速度，单位为：g  @~english  Acceleration in the Y direction of the gyroscope, unit: g */
	float az;						/**< @~chinese  陀螺仪Z方向加速度，单位为：g  @~english  Acceleration in the Z direction of the gyroscope, unit: g */
	float mx;						/**< @~chinese  陀螺仪X方向磁场，单位为：uT  @~english  The magnetic field in the X direction of the gyroscope, unit: uT */
	float my;						/**< @~chinese  陀螺仪Y方向磁场，单位为：uT  @~english  The magnetic field in the Y direction of the gyroscope, unit: uT */
	float mz;						/**< @~chinese  陀螺仪Z方向磁场，单位为：uT  @~english  The magnetic field in the Z direction of the gyroscope, unit: uT */
	float score;					/**< @~chinese  运动分数，范围为：[0-1], 数值越大越稳定  @~english  Sports score, the range is: [0-1], the larger the value, the more stable */
}GyroPositionAndPose;

#define ADDITION_RESERVED_LEN3	20
#define ADDITION_RESERVED_LEN	8
#define ADDITION_RESERVED_LEN2	4

typedef struct AdditionData
{
    char    reserved[ADDITION_RESERVED_LEN];
    KeyInfo keyInfo;
    char    reserved2[ADDITION_RESERVED_LEN2];
    int     timestamp;
	GyroPositionAndPose	gyroPosAndPos;
	char    reserved3[ADDITION_RESERVED_LEN3];
}AdditionData;

#define EXTRA_INFO_RESERVED_LEN	24

/**
* @~chinese
* @brief 帧的附加数据
* @~english
* @brief extra information of frame
**/
typedef union ExtraInfo
{
    AdditionData addition;
    char reserved[EXTRA_INFO_RESERVED_LEN];
}ExtraInfo;
#pragma pack(pop)

/**
* @~chinese
* @brief 流信息组合，用于打开流时使用，可通过ICamera::getStreamInfos获得
* @~english
* @brief stream information, returned by ICamera::getStreamInfos
**/
typedef struct StreamInfo
{
	STREAM_FORMAT format;	/**< @~chinese 流信息         @~english stream format*/ 
	int width;				/**< @~chinese 宽度           @~english stream width*/
	int height;				/**< @~chinese 高度           @~english stream height*/
	float fps;				/**< @~chinese 帧率           @~english stream framerate*/
}StreamInfo;

/**
* @~chinese
* @brief 相机信息，可通过ICamera::getInfo或ISystem::queryCameras获得
* @~english
* @brief camera informations, returned by ICamera::getStreamInfos or ISystem::queryCameras
**/
typedef struct CameraInfo
{
	char name[32];					/**< @~chinese 相机类型         @~english type of camera*/ 
	char serial[CAMERA_SERIAL_MAX];				/**< @~chinese 序列号           @~english serial number of camera*/
	char uniqueId[32];				/**< @~chinese 相机标识         @~english unique Id of camera*/
	char firmwareVersion[FIRMWARE_VERSION_MAX];		/**< @~chinese 固件版本         @~english version of firmware*/
	char algorithmVersion[ALGORITHM_VERSION_MAX];		/**< @~chinese 算法版本         @~english version of algorithm*/
}CameraInfo;

/**
* @~chinese
* @brief 相机内参
* @~english
* @brief Intrinsics of depth camera or RGB camera
**/
typedef struct Intrinsics
{
	short width;	/**< @~chinese 标定分辨率-宽度		@~english calibration resolution-width*/
	short height;	/**< @~chinese 标定分辨率-高度		@~english calibration resolution-height*/
	float fx;		/**< @~chinese 相机内参fx			@~english Intrinsics fx*/
	float zero01;	/**< @~chinese 相机内参zero01		@~english Intrinsics zero01*/
	float cx;		/**< @~chinese 相机内参cx			@~english Intrinsics cx*/
	float zeor10;	/**< @~chinese 相机内参zero10		@~english Intrinsics zero10*/
	float fy;		/**< @~chinese 相机内参fy			@~english Intrinsics fy*/
	float cy;		/**< @~chinese 相机内参cy			@~english Intrinsics cy*/
	float zeor20;	/**< @~chinese 相机内参zeor20		@~english Intrinsics zeor20*/
	float zero21;	/**< @~chinese 相机内参zeor21		@~english Intrinsics zeor21*/
	float one22;	/**< @~chinese 相机内参zeor22		@~english Intrinsics zeor22*/
}Intrinsics;

/**
* @~chinese
* @brief 深度相机到RGB相机间的旋转平移信息
* @~english
* @brief Rotation and translation offrom depth camera to RGB camera
**/
typedef struct Extrinsics
{
	float rotation[9];                           /**<@~chinese 3x3旋转矩阵      @~english column-major 3x3 rotation matrix */
	float translation[3];                        /**<@~chinese 3元素的平移矩阵  @~english three-element translation vector */
}Extrinsics;

//畸变参数个数
#define DISTORT_PARAM_CNT	5

/**
* @~chinese
* @brief 深度相机或RGB相机畸变参数
* @~english
* @brief Distort of depth camera or RGB camera
**/
typedef struct Distort
{
	float k1;
	float k2;
	float k3;
	float k4;
	float k5;
}Distort;

#ifdef __cplusplus
}
#endif

namespace cs {
	/**
	* @~chinese
	* @brief 二维点坐标
	* @~english
	* @brief 2D point coordinates
	**/
#ifndef POINT2F
#define POINT2F
	typedef struct Point2f
	{
		float x;
		float y;
	} Point2f;
#endif
	/**
	* @~chinese
	* @brief 三维点坐标
	* @~english
	* @brief 3D point coordinates
	**/
#ifndef POINT3F
#define POINT3F
	typedef struct Point3f
	{
		float x;
		float y;
		float z;
	} Point3f;
#endif
}

#endif
