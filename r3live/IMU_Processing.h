#ifndef __IMU_PROCESSING_H__
#define __IMU_PROCESSING_H__


#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include "internal/so3_math.h"
#include <Eigen/Eigen>
#include "internal/common_lib.h"
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <glog/logging.h>
// #include <fast_lio/States.h>
#include "omp.h"
/// *************Preconfiguration
#define MAX_INI_COUNT (20)
const inline bool time_list(BasePoint& x, BasePoint& y) { return (x.time < y.time); };
bool check_state(StatesGroup& state_inout);
void check_in_out_state(const StatesGroup& state_in, StatesGroup& state_inout);

/// *************IMU Process and undistortion
class ImuProcess
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		ImuProcess(double cov_omg_n, double cov_acc_n, double cov_gry_n, double cov_bias_acc_n, double cov_bias_gry_n);
	~ImuProcess();

	void Process(const MeasureGroup& meas, StatesGroup& state, BaseCloudPtr pcl_un_, bool useUndistort = true);
	void Reset();
	void IMU_Initial(const MeasureGroup& meas, StatesGroup& state, int& N);

	// Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt);

	void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr>& v_imu);

	void UndistortPcl(const MeasureGroup& meas, StatesGroup& state_inout, BaseCloud& pcl_in_out);
	void lic_state_propagate(const MeasureGroup& meas, StatesGroup& state_inout);
	void lic_point_cloud_undistort(const MeasureGroup& meas, const StatesGroup& state_inout, BaseCloud& pcl_out, bool is_static = false);
	StatesGroup imu_preintegration(const StatesGroup& state_inout, std::deque<ImuFrame>& v_imu, double end_pose_dt = 0);


	void Integrate(const sensor_msgs::ImuConstPtr& imu);
	void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr& lastimu);

	Eigen::Vector3d angvel_last;
	Eigen::Vector3d acc_s_last;

	Eigen::Matrix<double, DIM_OF_PROC_N, 1> cov_proc_noise;

	Eigen::Vector3d cov_acc;
	Eigen::Vector3d cov_gyr;
	Eigen::Vector3d Lidar_offset_to_IMU;
	Eigen::Matrix3d Lidar_R_to_IMU;

	// std::ofstream fout;

	double COV_OMEGA_NOISE_DIAG;
	double COV_ACC_NOISE_DIAG;
	double COV_GYRO_NOISE_DIAG;

	double COV_BIAS_ACC_NOISE_DIAG;
	double COV_BIAS_GYRO_NOISE_DIAG;

public:
	/*** Whether is the first frame, init for first frame ***/
	bool b_first_frame_ = true;
	bool imu_need_init_ = true;
	double scale_gravity;


	int init_iter_num = 1;
	Eigen::Vector3d mean_acc;
	Eigen::Vector3d mean_gyr;

	/*** Undistorted pointcloud ***/
	BaseCloudPtr cur_pcl_un_;

	//// For timestamp usage
	ImuFrame last_imu_;

	/*** For gyroscope integration ***/
	double start_timestamp_;
	/// Making sure the equal size: v_imu_ and v_rot_
	std::deque<sensor_msgs::ImuConstPtr> v_imu_;
	std::vector<Eigen::Matrix3d> v_rot_pcl_;
	std::vector<Pose6D> IMU_pose;
};
#endif // !__IMU_PROCESSING_H__