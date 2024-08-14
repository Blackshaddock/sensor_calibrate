#ifndef __VOXELMAPPING_H__
#define __VOXELMAPPING_H__

#include "voxel_map_util.h"
#include "boost/thread.hpp"
#include <glog/logging.h>
#include "pcl/filters/voxel_grid.h"
#include "opencv/cv.hpp"

#define LASER_POINT_COV (0.00015)

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
		double s_dStaticParam;

		//r3live
		double s_dFilterSizeMapMin;
		int	   s_iUpdatePointStep;
		double s_dMaximumKdtreeDis;
		double s_dPlanarCheckDis;
		double s_dRangDis;
		double s_dMaximumResDis;
		double s_dCubeLen;


		OdomConfig() :s_dRangeCov(0.002), s_dAngleCov(0.005), s_dGyrCovScale(1.0), s_dAccCovScale(1.0), s_iMaxIteration(10), s_iMaxPointsSize(100), s_iMaxCovPointsSize(100), s_iMaxLayer(2), s_dVoxelSize(1), s_dDownSampleSize(0.5), s_dPlannarThreshold(0.01), s_dStaticParam(0.005), s_dFilterSizeMapMin(0.4), s_iUpdatePointStep(1), s_dMaximumKdtreeDis(0.5), s_dPlanarCheckDis(0.1), s_dRangDis(500.), s_dMaximumResDis(0.3)
		, s_dCubeLen(1000000.){}
	};



	const int laserCloudWidth = 48;
	const int laserCloudHeight = 48;
	const int laserCloudDepth = 48;
	const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
	
	

	class VoxelMap {
	public:
		VoxelMap();
		bool sync_packages(MeasureGroup& meas);
		bool push_back(sc::LidarFrame& lidarframe);
		bool push_back(sc::ImuFrame& imuframe);
		bool start();
		bool run();
		bool runR3live();
		bool stop();
		bool JugeStatic(MeasureGroup &meas);
		void SaveVoxelMap(int id);
		void GetVoxelMap(const OctoTree* current_octo, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorPointCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloudPlane);
		void SavePosPath(std::string pospath);
		void lasermap_fov_segment();
		int get_cube_index(const int& i, const int& j, const int& k)
		{
			return (i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
		}
		


	private:
		OdomConfig::Ptr m_odoConfigPtr;
		V3D Lidar_offset_to_IMU;

		bool m_bLidarPush;                                           //判断激光数据是否push
		std::string m_sLogRootPath;
		vector<StatesGroup> m_vStatesGroup;
		KD_TREE m_iKdtree;
		shared_ptr<ImuProcess> m_pImu;
		//surf feature in map
		BaseCloudPtr featsFromMap;
		BaseCloudPtr cube_points_add;

		std::deque<sc::LidarFrame> m_dLidarBuffer;
		std::deque<sc::ImuFrame> m_dImuBuffer;
		std::unordered_map<VOXEL_LOC, OctoTree*> m_uVoxelMap;
		boost::thread* m_pThread;
		pcl::VoxelGrid<BasePoint> downSizeFilterSurf;
		std::mutex m_mlmutex, m_mimutex;
		BaseCloudPtr featsArray[laserCloudNum];
		bool cube_updated[laserCloudNum];
		bool _last_inFOV[laserCloudNum];
		bool now_inFOV[laserCloudNum];
		int laserCloudValidNum = 0;

		double deltaT, deltaR; //迭代优化过程中，比较两次的结果
		int laserCloudCenWidth = 24;
		int laserCloudCenHeight = 24;
		int laserCloudCenDepth = 24;
		Eigen::Vector3d position_last = V3D::Zero();
		Eigen::Vector3d XAxisPoint_body;
		Eigen::Vector3d XAxisPoint_world;
		MeasureGroup measures;
		std::vector<BoxPointType> cub_needrm;
		std::vector<BoxPointType> cub_needad;
		int FOV_RANGE = 4;
		StatesGroup g_lio_state;



	};



};


#endif // !__VOXELMAPPING_H__
