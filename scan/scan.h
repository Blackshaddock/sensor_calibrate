#include "../internal/Structs.h"
#include "../io/ReadPts.h"
#include <queue>
#include <mutex>
#include <ctime>
#include <omp.h>
#include <pcl/common/transforms.h>
#include "../internal/PoseD.h"
#include "internal/PlyIo.hpp"

#include <pcl/io/pcd_io.h>		// �����ڵ��ԣ���������ȡ��
#include <pcl/io/ply_io.h>
namespace scanframe
{
	using namespace sc;
	struct ScanFrameConfig
	{
		typedef std::shared_ptr<ScanFrameConfig> Ptr;
		bool useGroundConstraint;
		deviceType processType;
		Eigen::Matrix4d rotLidar2Motor;                       //�״�����ϵ������������ϵ
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
		/// ��Դ��б��������豸����������ת��������M0���ڵ�����ϵ
		/// </summary>
		/// <param name="cloudIn">
		/// ����ĵ���
		/// </param>
		void ConverPoints2M0(BaseCloudPtr cloudIn);


		/// <summary>
		/// ���Բ�ֵ�õ���ǰ�ǶȵĲ���ֵ
		/// </summary>
		/// <param name="angle">
		///	����Ƕ�
		/// </param>
		/// <returns>
		/// ��Ҫ�����ĽǶ�ֵ
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
		int threadNum_;                                       ///�߳���

		std::string debugProcessDataPath_;

	};

}