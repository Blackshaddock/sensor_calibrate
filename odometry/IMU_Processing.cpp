#include "IMU_Processing.h"
/// *************Preconfiguration



ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
    imu_en = true;
    init_iter_num = 1;
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    // old
    cov_acc_scale = V3D(1, 1, 1);
    cov_gyr_scale = V3D(1, 1, 1);

    cov_bias_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_acc = V3D(0.1, 0.1, 0.1);
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = V3D::Zero();
    Lid_offset_to_IMU << -0.0200, 0.0870, 0.1280;
    acc_s_last = V3D::Zero();
    Lid_rot_to_IMU << 0.9999, -0.0018, -0.0021, 0.0028, 0.7621, 0.64735, 0.0004, -0.6473, 0.76218;
    last_imu_.clear();
    b_first_frame_ = true;
    imu_need_init_ = true;
    
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = V3D::Zero();
    imu_need_init_ = true;
    start_timestamp_ = -1;
    init_iter_num = 1;
    v_imu_.clear();
    IMUpose.clear();
    cur_pcl_un_.reset(new BaseCloud());
}

void ImuProcess::set_extrinsic(const MD(4, 4)& T) {
    Lid_offset_to_IMU = T.block<3, 1>(0, 3);
    Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D& transl) {
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D& transl, const M3D& rot) {
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale(const V3D& scaler) {
    cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov_scale(const V3D& scaler) {
    cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D& b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D& b_a) { cov_bias_acc = b_a; }

void ImuProcess::IMU_init(const MeasureGroup& meas, StatesGroup& state_inout,
    int& N) {
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    std::cout << "IMU Initializing: " <<  double(N) / MAX_INI_COUNT * 100 << std::endl;
    V3D cur_acc, cur_gyr;

    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto& imu_acc = meas.imu.front().acc;
        const auto& gyr_acc = meas.imu.front().gyr;
        mean_acc << imu_acc[0], imu_acc[1], imu_acc[2];
        mean_gyr << gyr_acc[0], gyr_acc[1], gyr_acc[2];
        first_lidar_time = meas.lidar.startTime;
        // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
    }

    for (const auto& imu : meas.imu) {
        const auto& imu_acc = imu.acc; 
        const auto& gyr_acc = imu.gyr; 
        cur_acc << imu_acc[0], imu_acc[1], imu_acc[2];
        cur_gyr << gyr_acc[0], gyr_acc[1], gyr_acc[2];

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N +
            (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) *
            (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N +
            (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) *
            (N - 1.0) / (N * N);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        N++;
    }

    state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;

    state_inout.rot_end =
        M3D::Identity(); // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    state_inout.bias_g = mean_gyr;

    last_imu_ = meas.imu.back();
}



void ImuProcess::UndistortPcl(const MeasureGroup& meas,
    StatesGroup& state_inout,
    BaseCloud& pcl_out) {
    //StatesGroup state_inout = _state_inout;
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double& imu_beg_time = v_imu.front().time;
    const double& imu_end_time = v_imu.back().time;
    const double& pcl_beg_time = meas.lidar.startTime;

    /*** sort point clouds by offset time ***/
    pcl_out = *(meas.lidar.cloudPtr);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double& pcl_end_time = meas.lidar.endTime;
    

    /*** Initialize IMU pose ***/
    IMUpose.clear();
    // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end,
    // state.pos_end, state.rot_end));
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
    /*** forward propagation at each imu point ***/
    V3D acc_imu, angvel_avr, acc_avr;
    V3D vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);
    MD(DIM_STATE, DIM_STATE) F_x, cov_w;

    double dt = 0;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto&& head = *(it_imu);
        auto&& tail = *(it_imu + 1);

        if (tail.time < pcl_beg_time)
            continue;

        angvel_avr << 0.5 * (head.gyr[0] + tail.gyr[0]),
            0.5 * (head.gyr[1] + tail.gyr[1]),
            0.5 * (head.gyr[2] + tail.gyr[2]);
        acc_avr << 0.5 *
            (head.acc[0] + tail.acc[0]),
            0.5 * (head.acc[1] + tail.acc[1]),
            0.5 * (head.acc[2] + tail.acc[2]);


        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

        if (head.time < pcl_beg_time) {
            dt = tail.time - pcl_beg_time;
        }
        else {
            dt = tail.time - head.time;
        }

        /* covariance propagation */
        M3D acc_avr_skew;
        M3D Exp_f = Exp(angvel_avr, dt);
        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

        F_x.setIdentity();
        cov_w.setZero();

        F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
        F_x.block<3, 3>(0, 9) = -M3D::Identity() * dt;
        // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
        F_x.block<3, 3>(3, 6) = M3D::Identity() * dt;
        F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
        F_x.block<3, 3>(6, 12) = -R_imu * dt;
        F_x.block<3, 3>(6, 15) = M3D::Identity() * dt;

        //cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
        //cov_w.block<3, 3>(6, 6) =
        //    R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
        //cov_w.block<3, 3>(9, 9).diagonal() =
        //    cov_bias_gyr * dt * dt; // bias gyro covariance
        //cov_w.block<3, 3>(12, 12).diagonal() =
        //    cov_bias_acc * dt * dt; // bias acc covariance
        Eigen::Matrix3d Jr_omega_dt = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d cov_acc_diag, cov_gyr_diag, cov_omega_diag;
        cov_omega_diag = Eigen::Vector3d(0.1, 0.1, 0.1).asDiagonal();
        cov_acc_diag = Eigen::Vector3d(0.4, 0.4, 0.4).asDiagonal();
        cov_gyr_diag = Eigen::Vector3d(0.2, 0.2, 0.2).asDiagonal();
        // cov_w.block<3, 3>(0, 0) = cov_omega_diag * dt * dt;
        cov_w.block< 3, 3 >(0, 0) = Jr_omega_dt * cov_omega_diag * Jr_omega_dt * dt * dt;
        cov_w.block< 3, 3 >(3, 3) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt;
        cov_w.block< 3, 3 >(6, 6) = cov_acc_diag * dt * dt;
        cov_w.block< 3, 3 >(9, 9).diagonal() =
            Eigen::Vector3d(0.1, 0.1, 0.1) * dt * dt; // bias gyro covariance
        cov_w.block< 3, 3 >(12, 12).diagonal() =
            Eigen::Vector3d(0.05, 0.05, 0.05) * dt * dt;

        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

        /* propogation of IMU attitude */
        R_imu = R_imu * Exp_f;

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_imu * acc_avr + state_inout.gravity;

        /* propogation of IMU */
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU */
        vel_imu = vel_imu + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double&& offs_t = tail.time - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }
    
    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    state_inout.vel_end = vel_imu + note * acc_imu * dt;
    state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
    state_inout.pos_end =pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;


    //state_inout的值为IMU坐标系到全局坐标系的变换
    //p_w = T_wi * T_il * p_l

    auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;
    auto rot_liD_e = state_inout.rot_end * Lid_rot_to_IMU;

    /*** undistort each lidar point (backward propagation) ***/
    
    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        acc_imu << VEC_FROM_ARRAY(head->acc);
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        angvel_avr << VEC_FROM_ARRAY(head->gyr);

        for (; it_pcl->time  > head->offset_time; it_pcl--) {
            dt = it_pcl->time  -  head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is
             * represented in global frame */
            M3D R_i(R_imu * Exp(angvel_avr, dt));
            V3D T_i(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt);
            //计算的是相对于时间坐标系
            auto pos_liD_i = R_i * Lid_offset_to_IMU + T_i;
            auto rot_liD_i = R_i * Lid_rot_to_IMU;

            /*V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt +
                R_i * Lid_offset_to_IMU - pos_liD_e);*/

            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            //V3D P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);
            V3D P_compensate = state_inout.rot_end.transpose() *rot_liD_i * P_i + state_inout.rot_end.transpose()*(pos_liD_i - pos_liD_e);
            /// save Undistorted points and their rotation
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin())
                break;
        }
    }

}

// constant velocity model
void ImuProcess::only_propag(const MeasureGroup& meas, StatesGroup& state_inout,
    BaseCloudPtr& pcl_out) {
    const double& pcl_beg_time = meas.lidar.startTime;

    /*** sort point clouds by offset time ***/
    pcl_out = meas.lidar.cloudPtr;
    const double& pcl_end_time =
        pcl_beg_time + pcl_out->points.back().time ;

    MD(DIM_STATE, DIM_STATE) F_x, cov_w;
    double dt = 0;

    if (b_first_frame_) {
        dt = 0.1;
        b_first_frame_ = false;
        time_last_scan_ = pcl_beg_time;
    }
    else {
        dt = pcl_beg_time - time_last_scan_;
        time_last_scan_ = pcl_beg_time;
    }

    /* covariance propagation */
    // M3D acc_avr_skew;
    M3D Exp_f = Exp(state_inout.bias_g, dt);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
    F_x.block<3, 3>(0, 9) = M3D::Identity() * dt;
    F_x.block<3, 3>(3, 6) = M3D::Identity() * dt;
    cov_w.block<3, 3>(9, 9).diagonal() =
        cov_gyr * dt * dt; // for omega in constant model
    cov_w.block<3, 3>(6, 6).diagonal() =
        cov_acc * dt * dt; // for velocity in constant model

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
    state_inout.rot_end = state_inout.rot_end * Exp_f;
    state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;
}

void ImuProcess::lic_state_propagate(const MeasureGroup& meas, StatesGroup& state_inout)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    // const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double& imu_end_time = v_imu.back().time;
    const double& pcl_beg_time = meas.lidar.startTime;

    /*** sort point clouds by offset time ***/
    BaseCloud pcl_out = *(meas.lidar.cloudPtr);
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    const double& pcl_end_time = pcl_beg_time + pcl_out.points.back().time;
    double        end_pose_dt = pcl_end_time - imu_end_time;

    state_inout = imu_preintegration(state_inout, v_imu, end_pose_dt);
    last_imu_ = meas.imu.back();
}

StatesGroup ImuProcess::imu_preintegration(const StatesGroup& state_in, std::deque<ImuFrame>& v_imu, double end_pose_dt)
{
    //std::unique_lock< std::mutex > lock(g_imu_premutex);
    //StatesGroup                    state_inout = state_in;
    //
    //Eigen::Vector3d acc_imu(0, 0, 0), angvel_avr(0, 0, 0), acc_avr(0, 0, 0), vel_imu(0, 0, 0), pos_imu(0, 0, 0);
    //vel_imu = state_inout.vel_end;
    //pos_imu = state_inout.pos_end;
    //Eigen::Matrix3d R_imu(state_inout.rot_end);
    //Eigen::MatrixXd F_x(Eigen::Matrix< double, DIM_STATE, DIM_STATE >::Identity());
    //Eigen::MatrixXd cov_w(Eigen::Matrix< double, DIM_STATE, DIM_STATE >::Zero());
    //double          dt = 0;
    //int             if_first_imu = 1;
    //// printf("IMU start_time = %.5f, end_time = %.5f, state_update_time = %.5f, start_delta = %.5f\r\n", v_imu.front()->header.stamp.toSec() -
    //// g_lidar_star_tim,
    ////        v_imu.back()->header.stamp.toSec() - g_lidar_star_tim,
    ////        state_in.last_update_time - g_lidar_star_tim,
    ////        state_in.last_update_time - v_imu.front()->header.stamp.toSec());
    //for (std::deque< ImuFrame>::iterator it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
    //{
    //    // if(g_lidar_star_tim == 0 || state_inout.last_update_time == 0)
    //    // {
    //    //   return state_inout;
    //    // }
    //    auto&& head  = *(it_imu);
    //    auto&& tail  = *(it_imu + 1);

    //    angvel_avr << 0.5 * (head.gyr[0] + tail.gyr[0]), 0.5 * (head.gyr[1] + tail.gyr[1]),
    //        0.5 * (head.gyr[2] + tail.gyr[2]);
    //    acc_avr << 0.5 * (head.acc[0] + tail.acc[0]),
    //        0.5 * (head.acc[1] + tail.acc[1]), 0.5 * (head.acc[2] + tail.acc[2]);

    //    angvel_avr -= state_inout.bias_g;

    //    acc_avr = acc_avr - state_inout.bias_a;

    //    if (tail.time < state_inout.last_update_time)
    //    {
    //        continue;
    //    }

    //    if (if_first_imu)
    //    {
    //        if_first_imu = 0;
    //        dt = tail.time  - state_inout.last_update_time;
    //    }
    //    else
    //    {
    //        dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    //    }
    //    if (dt > 0.05)
    //    {
    //        dt = 0.05;
    //    }

    //    /* covariance propagation */
    //    Eigen::Matrix3d acc_avr_skew;
    //    Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
    //    acc_avr_skew << SKEW_SYM_MATRIX(acc_avr);
    //    // Eigen::Matrix3d Jr_omega_dt = right_jacobian_of_rotion_matrix<double>(angvel_avr*dt);
    //    Eigen::Matrix3d Jr_omega_dt = Eigen::Matrix3d::Identity();
    //    F_x.block< 3, 3 >(0, 0) = Exp_f.transpose();
    //    // F_x.block<3, 3>(0, 9) = -Eye3d * dt;
    //    F_x.block< 3, 3 >(0, 9) = -Jr_omega_dt * dt;
    //    // F_x.block<3,3>(3,0)  = -R_imu * off_vel_skew * dt;
    //    F_x.block< 3, 3 >(3, 3) = Eye3d; // Already the identity.
    //    F_x.block< 3, 3 >(3, 6) = Eye3d * dt;
    //    F_x.block< 3, 3 >(6, 0) = -R_imu * acc_avr_skew * dt;
    //    F_x.block< 3, 3 >(6, 12) = -R_imu * dt;
    //    F_x.block< 3, 3 >(6, 15) = Eye3d * dt;

    //    Eigen::Matrix3d cov_acc_diag, cov_gyr_diag, cov_omega_diag;
    //    cov_omega_diag = Eigen::Vector3d(COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG).asDiagonal();
    //    cov_acc_diag = Eigen::Vector3d(COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG).asDiagonal();
    //    cov_gyr_diag = Eigen::Vector3d(COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG).asDiagonal();
    //    // cov_w.block<3, 3>(0, 0) = cov_omega_diag * dt * dt;
    //    cov_w.block< 3, 3 >(0, 0) = Jr_omega_dt * cov_omega_diag * Jr_omega_dt * dt * dt;
    //    cov_w.block< 3, 3 >(3, 3) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt;
    //    cov_w.block< 3, 3 >(6, 6) = cov_acc_diag * dt * dt;
    //    cov_w.block< 3, 3 >(9, 9).diagonal() =
    //        Eigen::Vector3d(COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG) * dt * dt; // bias gyro covariance
    //    cov_w.block< 3, 3 >(12, 12).diagonal() =
    //        Eigen::Vector3d(COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG) * dt * dt; // bias acc covariance

    //    // cov_w.block<3, 3>(18, 18).diagonal() = Eigen::Vector3d(COV_NOISE_EXT_I2C_R, COV_NOISE_EXT_I2C_R, COV_NOISE_EXT_I2C_R) * dt * dt; // bias
    //    // gyro covariance cov_w.block<3, 3>(21, 21).diagonal() = Eigen::Vector3d(COV_NOISE_EXT_I2C_T, COV_NOISE_EXT_I2C_T, COV_NOISE_EXT_I2C_T) * dt
    //    // * dt;  // bias acc covariance cov_w(24, 24) = COV_NOISE_EXT_I2C_Td * dt * dt;

    //    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    //    R_imu = R_imu * Exp_f;
    //    acc_imu = R_imu * acc_avr - state_inout.gravity;
    //    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    //    vel_imu = vel_imu + acc_imu * dt;
    //    angvel_last = angvel_avr;
    //    acc_s_last = acc_imu;

    //    // cout <<  std::setprecision(3) << " dt = " << dt << ", acc: " << acc_avr.transpose()
    //    //      << " acc_imu: " << acc_imu.transpose()
    //    //      << " vel_imu: " << vel_imu.transpose()
    //    //      << " omega: " << angvel_avr.transpose()
    //    //      << " pos_imu: " << pos_imu.transpose()
    //    //       << endl;
    //    // cout << "Acc_avr: " << acc_avr.transpose() << endl;
    //}

    //
    //dt = end_pose_dt;

    //state_inout.last_update_time = v_imu.back()->header.stamp.toSec() + dt;
    //// cout << "Last update time = " <<  state_inout.last_update_time - g_lidar_star_tim << endl;
    //
    //state_inout.vel_end = vel_imu + acc_imu * dt;
    //state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
    //state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    //
    //
    //// cout << (state_inout - state_in).transpose() << endl;
    //return state_inout;
return state_in;
}



void ImuProcess::Process(const MeasureGroup& meas, StatesGroup& stat,
    BaseCloudPtr & cur_pcl_un_) {
    double t1, t2, t3;

    if (meas.imu.empty()) {
        return ;
    }
    
    assert(meas.lidar.cloudPtr != nullptr);
    if (imu_need_init_ ) {
        /// The very first lidar frame
        IMU_init(meas, stat, init_iter_num);

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        if (init_iter_num > MAX_INI_COUNT) {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;
            std::cout << "IMU Initials: Gravity:" <<  stat.gravity[0] << " " <<  stat.gravity[1]<< " " <<  stat.gravity[2] << " " << 
                mean_acc.norm() << " state.bias_g: " << cov_acc_scale[0] << " " <<  cov_acc_scale[1] << " " <<  cov_acc_scale[2] << " acc covarience : " << cov_acc[0] << " "<< cov_acc[1]<< " " << cov_acc[2] << " gry covarience : "  << cov_gyr[0] << " " << cov_gyr[1] << " " << cov_gyr[2] << std::endl;
            /*cov_acc = M3D::Identity() * cov_acc_scale;
            cov_gyr = M3D::Identity() * cov_gyr_scale;*/
           
            
        }

        return;
    }

    /// Undistort points： the first point is assummed as the base frame
    /// Compensate lidar points with IMU rotation (with only rotation now)
    if (imu_en) {
        cout << "Use IMU" << endl;
        UndistortPcl(meas, stat, *cur_pcl_un_);
        last_imu_ = meas.imu.back();
    }
    else {
        cout << "No IMU, use constant velocity model" << endl;
        cov_acc = M3D::Identity() * cov_acc_scale;
        cov_gyr = M3D::Identity() * cov_gyr_scale;
        only_propag(meas, stat, cur_pcl_un_);
    }
    //lic_state_propagate(meas, stat);
}

const bool time_list(BasePoint& x, BasePoint& y)
{
        return (x.time < y.time);
}
