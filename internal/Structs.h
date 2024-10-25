#ifndef __STRUCTS_H__
#define __STRUCTS_H__

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>
#include <opencv/cv.hpp>
#include "internal/PoseD.h"

// namespace of sensor calibration
namespace sc {




struct EIGEN_ALIGN16 PointXYZIRT {
	PCL_ADD_POINT4D;
	PCL_ADD_NORMAL4D;
	double time;
	float angle;		// �������Ƕ�
	uint8_t ring;
	int32_t id;
	union {
		struct {
			float intensity;
		};
		float data_c[4];
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

typedef PointXYZIRT					BasePoint;
typedef pcl::PointCloud<BasePoint>	BaseCloud;
typedef BaseCloud::Ptr				BaseCloudPtr;



struct PointXYZIRGB {
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	float intensity;
	uint32_t id;
	int      isColored;
	int      picNum;
	int      u;
	int      v;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

typedef PointXYZIRGB				ColorPoint;
typedef pcl::PointCloud<ColorPoint>	ColorCloud;
typedef ColorCloud::Ptr				ColorCloudPtr;

struct LidarFrame {
	double startTime;
	double endTime;
	int frameId;
	BaseCloudPtr cloudPtr;

	LidarFrame() : startTime(0.), endTime(0.), 
				   frameId(0), cloudPtr(new BaseCloud) {}

	~LidarFrame() {
		this->startTime = 0;
		this->endTime = 0;
		this->frameId = 0;
		this->cloudPtr = nullptr;
	}

	LidarFrame(const LidarFrame& other)
	{
		this->startTime = other.startTime;
		this->endTime = other.endTime;
		this->frameId = other.frameId;
		this->cloudPtr = other.cloudPtr;
		//other.~LidarFrame();
	}

	LidarFrame(LidarFrame&& other) {
		this->startTime = other.startTime;
		this->endTime = other.endTime;
		this->frameId = other.frameId;
		this->cloudPtr = other.cloudPtr;
		other.~LidarFrame();
	}

	void operator=(LidarFrame&& other) {
		this->startTime = other.startTime;
		this->endTime = other.endTime;
		this->frameId = other.frameId;
		this->cloudPtr = other.cloudPtr;
		other.~LidarFrame();
	}

	void operator=(LidarFrame& other) {
		this->startTime = other.startTime;
		this->endTime = other.endTime;
		this->frameId = other.frameId;
		this->cloudPtr = other.cloudPtr;
	}

	void Clear() {
		startTime = 0.;
		endTime = 0.;
		frameId = 0;
		cloudPtr->clear();
	}
};





struct ImuFrame
{
	float acc[3];
	float gyr[3];
	double time;
	ImuFrame() : acc{ 0.0, 0.0, 0.0 }, gyr{0.0, 0.0, 0.0}, time(0.0) {}
	void clear()
	{
		std::fill_n(acc, 3, 0.0);
		std::fill_n(gyr, 3, 0.0);
		time = 0.0;
	}
};


struct MotorCalibMatchPair {
	BasePoint sPt;
	BasePoint tPt;
	bool isValid;

	MotorCalibMatchPair() : isValid(false) {}
};
typedef std::vector<MotorCalibMatchPair> MotorCalibMatchPairs;


struct MeasureGroup // Lidar data and imu dates for the curent process
{
	LidarFrame lidar;
	std::deque<ImuFrame> imu;
	bool isStatic;                   // ��֡�����Ƿ�Ϊ��ֹ
	MeasureGroup() : isStatic(false) {}
};

struct ImageFrame {
	cv::Mat image;
	double  time;
	PoseD   pose;
	int     frameId;
	int     imgtype;    //0   1   2

};

};// namespace sc

POINT_CLOUD_REGISTER_POINT_STRUCT(sc::PointXYZIRT, (float, x, x)
								 (float, y, y)
								 (float, z, z)
								 (float, normal_x, normal_x)
								 (float, normal_y, normal_y)
								 (float, normal_z, normal_z)
								 (float, intensity, intensity)
								 (double, time, time)
								 (float, angle, angle)
								 (int32_t, id, id)
								 (uint8_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(sc::PointXYZIRGB, (float, x, x)
								 (float, y, y)
								 (float, z, z)
								 (float, intensity, intensity)
								 (uint8_t, r, r)
								 (uint8_t, g, g)
								 (uint8_t, b, b)
								 (uint32_t, id, id)
								 (int, isColored, isColored)
								 (int, picNum, picNum)
								 (int, u, u)
								 (int, v, v)
								)

#endif