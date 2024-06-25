#ifndef __VOXELMAPPING_H__
#define __VOXELMAPPING_H__

#include "voxel_map_util.h"
#include "boost/thread.hpp"
#include <glog/logging.h>
#include "pcl/filters/voxel_grid.h"

namespace odom {


	struct OdomConfig {
		typedef std::shared_ptr<OdomConfig> Ptr;
		//Noise Param
		double s_dRangeCov;
		double s_dAngleCov;
		double s_dGyrCovScale;
		double s_dAccCovScale;

		//R/T
		vector<double> s_vdExtrinsicT;         //imu激光器外参T
		vector<double> s_vdExtrinsicR;		   //imu激光器外参R
		vector<double> s_vdLayerPointSize;
		std::vector<int> s_vLayerSize;

		//mapping algorithm params
		int s_iMaxIteration;
		int s_iMaxPointsSize;
		int s_iMaxCovPointsSize;
		int s_iMaxLayer;
		double s_dVoxelSize;
		double s_dDownSampleSize;
		double s_dPlannarThreshold;
		


		OdomConfig() :s_dRangeCov(0.02), s_dAngleCov(0.05), s_dGyrCovScale(1.0), s_dAccCovScale(1.0), s_iMaxIteration(4), s_iMaxPointsSize(100), s_iMaxCovPointsSize(100), s_iMaxLayer(2), s_dVoxelSize(1.0), s_dDownSampleSize(0.5), s_dPlannarThreshold(0.01){}
	};

	class VoxelMap {
	public:
		VoxelMap();
		bool sync_packages(MeasureGroup& meas);
		bool push_back(sc::LidarFrame& lidarframe);
		bool push_back(sc::ImuFrame& imuframe);
		bool start();
		bool run();
		bool stop();


	private:
		OdomConfig::Ptr m_odoConfigPtr;
		V3D Lidar_offset_to_IMU;

		bool m_bLidarPush;                                           //判断激光数据是否push

		shared_ptr<ImuProcess> m_pImu;
		std::deque<sc::LidarFrame> m_dLidarBuffer;
		std::deque<sc::ImuFrame> m_dImuBuffer;
		std::unordered_map<VOXEL_LOC, OctoTree*> m_uVoxelMap;
		boost::thread* m_pThread;
		pcl::VoxelGrid<BasePoint> downSizeFilterSurf;





	};



};


#endif // !__VOXELMAPPING_H__
