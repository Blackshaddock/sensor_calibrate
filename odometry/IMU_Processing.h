#ifndef __IMU_PROCESSING_H__
#define __IMU_PROCESSING_H__

#include <Eigen/Eigen>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <mutex>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include "common_lib.h"
#include "internal/PlyIo.hpp"
/// *************Preconfiguration

#define MAX_INI_COUNT (200)




// *************IMU Process and undistortion
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ImuProcess();
    ~ImuProcess();

    void Reset();
    void Reset(double start_timestamp, const ImuFrame& lastimu);
    void set_extrinsic(const V3D& transl, const M3D& rot);
    void set_extrinsic(const V3D& transl);
    void set_extrinsic(const MD(4, 4)& T);
    void set_gyr_cov_scale(const V3D& scaler);
    void set_acc_cov_scale(const V3D& scaler);
    void set_gyr_bias_cov(const V3D& b_g);
    void set_acc_bias_cov(const V3D& b_a);
    void Process(const MeasureGroup& meas, StatesGroup& state,
        BaseCloudPtr& pcl_un_);

    ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    M3D Lid_rot_to_IMU;
    V3D Lid_offset_to_IMU;
    double first_lidar_time;
    bool imu_en;

private:
    void IMU_init(const MeasureGroup& meas, StatesGroup& state, int& N);
    void UndistortPcl(const MeasureGroup& meas, StatesGroup& state_inout,
        BaseCloud& pcl_in_out);
    void only_propag(const MeasureGroup& meas, StatesGroup& state_inout,
        BaseCloudPtr& pcl_out);

    void SaveLidarFrame(BaseCloudPtr  basecloud);

    ImuFrame last_imu_;
    deque<ImuFrame> v_imu_;
    BaseCloudPtr cur_pcl_un_;
    vector<Pose6D> IMUpose;
    vector<M3D> v_rot_pcl_;

    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double start_timestamp_;
    double time_last_scan_;
    int init_iter_num = 1;
    bool b_first_frame_ ;
    bool imu_need_init_ ;
};

const bool time_list(BasePoint& x, BasePoint& y);

#endif