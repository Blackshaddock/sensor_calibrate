#include "voxelMapping.h"

namespace odom {


	const bool var_contrast(pointWithCov& x, pointWithCov& y) {
		return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
	};

	VoxelMap::VoxelMap()
	{
		Lidar_offset_to_IMU = V3D::Zero();
		m_pThread = nullptr;
		m_odoConfigPtr  = std::make_shared<OdomConfig>();
		m_odoConfigPtr->s_vdLayerPointSize.push_back(5);
		m_odoConfigPtr->s_vdLayerPointSize.push_back(5);
		m_odoConfigPtr->s_vdLayerPointSize.push_back(5);
		m_odoConfigPtr->s_vdLayerPointSize.push_back(5);
		m_odoConfigPtr->s_vdLayerPointSize.push_back(5);
		for (int i = 0; i < m_odoConfigPtr->s_vdLayerPointSize.size(); i++)
			m_odoConfigPtr->s_vLayerSize.push_back(m_odoConfigPtr->s_vdLayerPointSize[i]);
		m_pImu = std::make_shared<ImuProcess>();
		downSizeFilterSurf.setLeafSize(m_odoConfigPtr->s_dDownSampleSize, m_odoConfigPtr->s_dDownSampleSize, m_odoConfigPtr->s_dDownSampleSize);
	}

	bool VoxelMap::sync_packages(MeasureGroup& meas)
	{

		if (m_dLidarBuffer.empty() || m_dImuBuffer.empty())
		{
			LOG(INFO) << "The Data is empty, please wait!" << std::endl;
			return 0;
		}

		//push lidar scan
		if (!m_bLidarPush)
		{
			meas.lidar = m_dLidarBuffer.front();
			if (meas.lidar.cloudPtr->points.size() <= 1)
			{
				return false;
			}
			m_bLidarPush = true;
		}
		if (meas.lidar.endTime > m_dImuBuffer.back().time)
			return false;

		//push imu data
		auto imu_time = m_dImuBuffer.front().time;
		meas.imu.clear();
		while (!m_dImuBuffer.empty() && imu_time < meas.lidar.endTime)
		{
			imu_time = m_dImuBuffer.front().time;
			if (imu_time > meas.lidar.endTime + 0.005)
			{
				break;
			}
			meas.imu.push_back(m_dImuBuffer.front());
			m_dImuBuffer.pop_front();
		}
		m_bLidarPush = false;
		m_dLidarBuffer.pop_front();
		return 1;
	}

	bool VoxelMap::push_back(sc::LidarFrame& lidarframe)
	{
		m_dLidarBuffer.push_back(lidarframe);
		return true;
	}

	bool VoxelMap::push_back(sc::ImuFrame& imuframe)
	{
		m_dImuBuffer.push_back(imuframe);
		return true;
	}

	bool VoxelMap::start()
	{
		stop();
		m_pThread = new boost::thread(boost::bind(&VoxelMap::run, this));
		return true;
	}

	bool VoxelMap::run()
	{

		VD(DIM_STATE) solution;
		MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
		V3D rot_add, t_add;
		double deltaT, deltaR;
		V3D euler_cur;
		V3D position_last(V3D::Zero());

		BaseCloudPtr feats_undistort(new BaseCloud());
		BaseCloudPtr feats_down_body(new BaseCloud());

		BaseCloudPtr normvec(new BaseCloud);
		BaseCloudPtr laserCloudOri(new BaseCloud);
		BaseCloudPtr laserCloudNoeffect(new BaseCloud);
		BaseCloudPtr corr_normvect(new BaseCloud);

		MeasureGroup measures;
		StatesGroup state, state_propagat;
		bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0,
			is_first_frame = true;
		double first_lidar_time = 0;
		bool init_map = false;
		double total_distance = 0;
		while (true)
		{

			bool nearest_search_en = true;
			int rematch_num = 0;
			
			if (m_dImuBuffer.empty() || m_dLidarBuffer.empty())
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			
			if (sync_packages(measures))
			{
				LOG(INFO) << std::setprecision(14) << "imu size: " << measures.imu.size() << " " << measures.imu.back().time << " " << measures.lidar.endTime << " " << measures.imu.front().time  <<  std::endl;
				auto undistort_start = std::chrono::high_resolution_clock::now();
				m_pImu->Process(measures, state, feats_undistort);
				auto undistort_end = std::chrono::high_resolution_clock::now();
				auto undistort_time = std::chrono::duration_cast<std::chrono::duration<double>>(undistort_end - undistort_start).count() * 1000;


				state_propagat = state;

				if (is_first_frame) {
					first_lidar_time = measures.lidar.startTime;
					is_first_frame = false;
				}

				if (feats_undistort->empty() || (feats_undistort == NULL)) {
					m_pImu->first_lidar_time = first_lidar_time;
					cout << "FAST-LIO not ready" << endl;
					continue;
				}
				flg_EKF_inited = (measures.lidar.startTime - first_lidar_time) < 0.0 ? false : true;

				//构建初始地图
				if (flg_EKF_inited && !init_map) {
					BaseCloudPtr world_lidar(new BaseCloud);
					Eigen::Quaterniond q(state.rot_end);
					transformLidar(state, m_pImu, feats_undistort, world_lidar);
					std::vector<pointWithCov> pv_list;
					for (size_t i = 0; i < world_lidar->size(); i++) {
						pointWithCov pv;
						pv.point << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
						V3D point_this(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);
						// if z=0, error will occur in calcBodyCov. To be solved
						if (point_this[2] == 0) {
							point_this[2] = 0.001;
						}
						M3D cov;
						calcBodyCov(point_this, m_odoConfigPtr->s_dRangeCov, m_odoConfigPtr->s_dAngleCov, cov);

						point_this += Lidar_offset_to_IMU;
						M3D point_crossmat;
						point_crossmat << SKEW_SYM_MATRX(point_this);
						cov = state.rot_end * cov * state.rot_end.transpose() +
							(-point_crossmat) * state.cov.block<3, 3>(0, 0) *
							(-point_crossmat).transpose() +
							state.cov.block<3, 3>(3, 3);
						pv.cov = cov;
						pv_list.push_back(pv);
						Eigen::Vector3d sigma_pv = pv.cov.diagonal();
						sigma_pv[0] = sqrt(sigma_pv[0]);
						sigma_pv[1] = sqrt(sigma_pv[1]);
						sigma_pv[2] = sqrt(sigma_pv[2]);
					}

					buildVoxelMap(pv_list, m_odoConfigPtr->s_dVoxelSize, m_odoConfigPtr->s_iMaxLayer, m_odoConfigPtr->s_vLayerSize,
						m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_dPlannarThreshold,
						m_uVoxelMap);
					LOG(INFO) << "build voxel map" << std::endl;
					init_map = true;
					continue;
				}

				downSizeFilterSurf.setInputCloud(feats_undistort);
				downSizeFilterSurf.filter(*feats_down_body);
				sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);

				std::vector<M3D> body_var;
				std::vector<M3D> crossmat_list;
				double total_residual = 0.0;


				/*** iterated state estimation ***/
				//将noise考虑进去，并计算每个点的坐标
				auto calc_point_cov_start = std::chrono::high_resolution_clock::now();
				for (size_t i = 0; i < feats_down_body->size(); i++) {
					V3D point_this(feats_down_body->points[i].x,
						feats_down_body->points[i].y,
						feats_down_body->points[i].z);
					if (point_this[2] == 0) {
						point_this[2] = 0.001;
					}
					M3D cov;
					calcBodyCov(point_this, m_odoConfigPtr->s_dRangeCov, m_odoConfigPtr->s_dAngleCov, cov);
					M3D point_crossmat;
					point_crossmat << SKEW_SYM_MATRX(point_this);
					crossmat_list.push_back(point_crossmat);
					M3D rot_var = state.cov.block<3, 3>(0, 0);
					M3D t_var = state.cov.block<3, 3>(3, 3);
					body_var.push_back(cov);
				}

				for (int iterCount = 0; iterCount < m_odoConfigPtr->s_iMaxIteration; iterCount++)
				{
					std::vector<ptpl> ptpl_list;


					BaseCloudPtr world_lidar(new BaseCloud);
					transformLidar(state, m_pImu, feats_down_body, world_lidar);

					vector<pointWithCov> pv_list;
					std::vector<M3D> var_list;
					for (size_t i = 0; i < feats_down_body->size(); i++) {
						pointWithCov pv;
						pv.point << feats_down_body->points[i].x,
							feats_down_body->points[i].y, feats_down_body->points[i].z;
						pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y,
							world_lidar->points[i].z;
						M3D cov = body_var[i];
						M3D point_crossmat = crossmat_list[i];
						M3D rot_var = state.cov.block<3, 3>(0, 0);
						M3D t_var = state.cov.block<3, 3>(3, 3);
						cov = state.rot_end * cov * state.rot_end.transpose() +
							(-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
							t_var;
						pv.cov = cov;
						pv_list.push_back(pv);
						var_list.push_back(cov);
					}

					std::vector<V3D> non_match_list;
					BuildResidualListOMP(m_uVoxelMap, m_odoConfigPtr->s_dVoxelSize, 3.0, m_odoConfigPtr->s_iMaxLayer, pv_list,
						ptpl_list, non_match_list);


					int effct_feat_num = 0;
					for (int i = 0; i < ptpl_list.size(); i++) {
						BasePoint pi_body;
						BasePoint pi_world;
						BasePoint pl;
						pi_body.x = ptpl_list[i].point(0);
						pi_body.y = ptpl_list[i].point(1);
						pi_body.z = ptpl_list[i].point(2);
						pointBodyToWorld(state, &pi_body, &pi_world);
						pl.x = ptpl_list[i].normal(0);
						pl.y = ptpl_list[i].normal(1);
						pl.z = ptpl_list[i].normal(2);
						effct_feat_num++;
						float dis = (pi_world.x * pl.x + pi_world.y * pl.y +
							pi_world.z * pl.z + ptpl_list[i].d);
						pl.intensity = dis;
						laserCloudOri->push_back(pi_body);
						corr_normvect->push_back(pl);
						total_residual += fabs(dis);
					}

					/*** Computation of Measuremnt Jacobian matrix H and measurents vector* ***/
					MatrixXd Hsub(effct_feat_num, 6);
					MatrixXd Hsub_T_R_inv(6, effct_feat_num);
					VectorXd R_inv(effct_feat_num);
					VectorXd meas_vec(effct_feat_num);

					for (int i = 0; i < effct_feat_num; i++) {
						const BasePoint& laser_p = laserCloudOri->points[i];
						V3D point_this(laser_p.x, laser_p.y, laser_p.z);
						M3D cov;

						cov = state.rot_end * cov * state.rot_end.transpose();
						M3D point_crossmat;
						point_crossmat << SKEW_SYM_MATRX(point_this);
						const BasePoint& norm_p = corr_normvect->points[i];
						V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
						V3D point_world = state.rot_end * point_this + state.pos_end;
						// /*** get the normal vector of closest surface/corner ***/
						Eigen::Matrix<double, 1, 6> J_nq;
						J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
						J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
						double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
						R_inv(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
						double ranging_dis = point_this.norm();
						laserCloudOri->points[i].intensity = sqrt(R_inv(i));
						laserCloudOri->points[i].normal_x =
							corr_normvect->points[i].intensity;
						laserCloudOri->points[i].normal_y = sqrt(sigma_l);
						laserCloudOri->points[i].normal_z =
							sqrt(norm_vec.transpose() * cov * norm_vec);
						laserCloudOri->points[i].time =
							sqrt(sigma_l + norm_vec.transpose() * cov * norm_vec);

						/*** calculate the Measuremnt Jacobian matrix H ***/
						V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
						Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
						Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
							A[2] * R_inv(i), norm_p.x* R_inv(i), norm_p.y* R_inv(i),
							norm_p.z* R_inv(i);
						/*** Measuremnt: distance to the closest surface/corner ***/
						meas_vec(i) = -norm_p.intensity;
					}

					MatrixXd K(DIM_STATE, effct_feat_num);

					EKF_stop_flg = false;
					flg_EKF_converged = false;

					/*** Iterative Kalman Filter Update ***/
					if (!flg_EKF_inited) {
						cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
						/*** only run in initialization period ***/
						MatrixXd H_init(MD(9, DIM_STATE)::Zero());
						MatrixXd z_init(VD(9)::Zero());
						H_init.block<3, 3>(0, 0) = M3D::Identity();
						H_init.block<3, 3>(3, 3) = M3D::Identity();
						H_init.block<3, 3>(6, 15) = M3D::Identity();
						z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
						z_init.block<3, 1>(0, 0) = -state.pos_end;

						auto H_init_T = H_init.transpose();
						auto&& K_init =
							state.cov * H_init_T *
							(H_init * state.cov * H_init_T + 0.0001 * MD(9, 9)::Identity())
							.inverse();
						solution = K_init * z_init;

						state.resetpose();
						EKF_stop_flg = true;
					}
					else {
						auto&& Hsub_T = Hsub.transpose();
						H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
						MD(DIM_STATE, DIM_STATE) && K_1 =
							(H_T_H + (state.cov).inverse()).inverse();
						K = K_1.block<DIM_STATE, 6>(0, 0) * Hsub_T_R_inv;
						auto vec = state_propagat - state;
						solution = K * meas_vec + vec - K * Hsub * vec.block<6, 1>(0, 0);

						int minRow, minCol;
						if (0) // if(V.minCoeff(&minRow, &minCol) < 1.0f)
						{
							VD(6) V = H_T_H.block<6, 6>(0, 0).eigenvalues().real();
							cout << "!!!!!! Degeneration Happend, eigen values: "
								<< V.transpose() << endl;
							EKF_stop_flg = true;
							solution.block<6, 1>(9, 0).setZero();
						}

						state += solution;

						rot_add = solution.block<3, 1>(0, 0);
						t_add = solution.block<3, 1>(3, 0);

						if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) {
							flg_EKF_converged = true;
						}

						deltaR = rot_add.norm() * 57.3;
						deltaT = t_add.norm() * 100;
					}
					euler_cur = RotMtoEuler(state.rot_end);
					/*** Rematch Judgement ***/
					nearest_search_en = false;
					if (flg_EKF_converged ||
						((rematch_num == 0) && (iterCount == (m_odoConfigPtr->s_iMaxIteration - 2)))) {
						nearest_search_en = true;
						rematch_num++;
					}

					/*** Convergence Judgements and Covariance Update ***/
					if (!EKF_stop_flg &&
						(rematch_num >= 2 || (iterCount == m_odoConfigPtr->s_iMaxIteration - 1))) {
						if (flg_EKF_inited) {
							/*** Covariance Update ***/
							G.setZero();
							G.block<DIM_STATE, 6>(0, 0) = K * Hsub;
							state.cov = (I_STATE - G) * state.cov;
							total_distance += (state.pos_end - position_last).norm();
							position_last = state.pos_end;

							VD(DIM_STATE) K_sum = K.rowwise().sum();
							VD(DIM_STATE) P_diag = state.cov.diagonal();
						}
						EKF_stop_flg = true;
					}
					if (EKF_stop_flg)
						break;
				}

				/*** add the  points to the voxel map ***/
				auto map_incremental_start = std::chrono::high_resolution_clock::now();
				BaseCloudPtr world_lidar(
					new BaseCloud);
				transformLidar(state, m_pImu, feats_down_body, world_lidar);
				std::vector<pointWithCov> pv_list;
				for (size_t i = 0; i < world_lidar->size(); i++) {
					pointWithCov pv;
					pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
						world_lidar->points[i].z;
					M3D point_crossmat = crossmat_list[i];
					M3D cov = body_var[i];
					cov = state.rot_end * cov * state.rot_end.transpose() +
						(-point_crossmat) * state.cov.block<3, 3>(0, 0) *
						(-point_crossmat).transpose() +
						state.cov.block<3, 3>(3, 3);
					pv.cov = cov;
					pv_list.push_back(pv);
				}
				std::sort(pv_list.begin(), pv_list.end(), var_contrast);
				updateVoxelMap(pv_list, m_odoConfigPtr->s_dVoxelSize, m_odoConfigPtr->s_iMaxLayer, m_odoConfigPtr->s_vLayerSize,
					m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_dPlannarThreshold,
					m_uVoxelMap);

			}
		}

		return false;
	}

	bool VoxelMap::stop()
	{
		if (m_pThread) {
			m_pThread->join();
			delete m_pThread;
			m_pThread = NULL;
		}
		return false;
	}
}