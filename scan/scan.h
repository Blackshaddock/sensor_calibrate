#include "../internal/Structs.h"
#include "../io/ReadPts.h"
#include <queue>
#include <mutex>
#include <ctime>
#include <omp.h>
#include <pcl/common/transforms.h>
#include "../internal/PoseD.h"
#include "internal/PlyIo.hpp"

#include <pcl/io/pcd_io.h>		// 仅用于调试，后续可以取消
#include <pcl/io/ply_io.h>
namespace scanframe
{
	using namespace sc;
	struct ScanFrameConfig
	{
		typedef std::shared_ptr<ScanFrameConfig> Ptr;
		bool useGroundConstraint;
		deviceType processType;
		Eigen::Matrix4d rotLidar2Motor;                       //雷达坐标系到编码器坐标系
		double alphaX;
		double alphaY;
		double alphaZ;
		Eigen::Vector3d dp;
		std::vector<double> correctAngles;
		std::string correctAngleFile;
		bool LoadAngleCorrectionFile();
		
	};
	class ScanFrame
	{
	public:
		typedef std::shared_ptr<ScanFrame> Ptr;
		ScanFrame() {
			threadNum_ = omp_get_max_threads() * 0.8;
			LOG(INFO) << "Thread use: " << threadNum_;
		};
		ScanFrame(SenSorParamsConfig::Ptr sensorParamsConfig, ScanFrameConfig::Ptr scanFrameConfig);
		void InputLidarFrame(LidarFrame & lidarFrame);
		void InputImuFrame(ImuFrame& imuFrame);

		/// <summary>
		/// 针对带有编码器的设备，将激光器转到编码器M0所在的坐标系
		/// </summary>
		/// <param name="cloudIn">
		/// 传入的点云
		/// </param>
		void ConverPoints2M0(BaseCloudPtr cloudIn);


		/// <summary>
		/// 线性插值得到当前角度的补偿值
		/// </summary>
		/// <param name="angle">
		///	传入角度
		/// </param>
		/// <returns>
		/// 需要补偿的角度值
		/// </returns>
		double Slerp(double angle);


		bool Run();
		
	private:
		ScanFrameConfig::Ptr scanFrameConfig_;
		std::queue<LidarFrame> qLidarFrames_;
		std::queue<ImuFrame> qImuFrames_;
		SendataIo::Ptr gSensorData_;
		std::mutex m_LidarFrames_;
		std::mutex m_ImuFrames_;

		LidarFrame curLidarFrame_;
		int threadNum_;                                       ///线程数

		std::string debugProcessDataPath_;

	};

}