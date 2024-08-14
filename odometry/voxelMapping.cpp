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
		m_sLogRootPath = "E:\\data\\RTK\\RTK\\ProcessData\\";
		deltaT = 0.0 , deltaR = 0.;
		XAxisPoint_body  = Eigen::Vector3d(LIDAR_SP_LEN, 0., 0.);
		XAxisPoint_world = Eigen::Vector3d(LIDAR_SP_LEN, 0., 0.);

		//surf feature in map
		featsFromMap.reset(new BaseCloud());
		cube_points_add.reset(new BaseCloud());
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
			m_mimutex.lock();
			m_dImuBuffer.pop_front();
			m_mimutex.unlock();
		}
		m_bLidarPush = false;
		m_mlmutex.lock();
		m_dLidarBuffer.pop_front();
		m_mlmutex.unlock();
		return 1;
	}

	bool VoxelMap::push_back(sc::LidarFrame& lidarframe)
	{
		m_mlmutex.lock();
		m_dLidarBuffer.push_back(lidarframe);
		m_mlmutex.unlock();
		return true;
	}

	bool VoxelMap::push_back(sc::ImuFrame& imuframe)
	{
		m_mimutex.lock();
		m_dImuBuffer.push_back(imuframe);
		m_mimutex.unlock();
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
		G.setZero();
		H_T_H.setZero();
		I_STATE.setIdentity();
		V3D rot_add, t_add;
		
		V3D euler_cur;
		V3D position_last(V3D::Zero());

		BaseCloudPtr feats_undistort(new BaseCloud());
		BaseCloudPtr feats_down_body(new BaseCloud());

		BaseCloudPtr normvec(new BaseCloud);
		BaseCloudPtr laserCloudOri(new BaseCloud);
		BaseCloudPtr laserCloudNoeffect(new BaseCloud);
		BaseCloudPtr corr_normvect(new BaseCloud);

		
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
				std::this_thread::sleep_for(std::chrono::microseconds(8));
				continue;
			}
			
			if (sync_packages(measures))
			{
				JugeStatic(measures);
				m_vStatesGroup.push_back(state);
				SavePosPath(m_sLogRootPath  + "_Framepos.txt");
				LOG(INFO) << std::setprecision(14) << "imu size: " << measures.imu.size() << " " << measures.imu.back().time << " " << measures.lidar.endTime << " " << measures.imu.front().time  << " " << m_vStatesGroup.size() <<  std::endl;
				if(!measures.isStatic)
					m_pImu->Process(measures, state, feats_undistort);
				else
				{
					//将点云转到IMU坐标系
					transformLidar(state, m_pImu, measures.lidar.cloudPtr, feats_undistort, 1);
					if(m_pImu->imu_need_init_)
						m_pImu->Process(measures, state, feats_undistort);
					if (init_map)
					{
						std::vector<pointWithCov> vPointCov;
						M3D cov; pointWithCov ptCov;
						for (auto& pt : feats_undistort->points)
						{
							Eigen::Vector3d this_point(pt.x, pt.y, pt.z);
							calcBodyCov(this_point, m_odoConfigPtr->s_dRangeCov, m_odoConfigPtr->s_dAngleCov, cov);
							ptCov.point = this_point;
							ptCov.point_world = this_point;
							ptCov.cov = cov;
							vPointCov.push_back(ptCov);
						}
						updateVoxelMap(vPointCov, m_odoConfigPtr->s_dVoxelSize, m_odoConfigPtr->s_iMaxLayer, m_odoConfigPtr->s_vLayerSize,
							m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_dPlannarThreshold,
							m_uVoxelMap);
						/*if(measures.lidar.frameId %10 == 0)*/
						SaveVoxelMap(measures.lidar.frameId);
						continue;
					}
					

				}
				PlyIo fullOriginPly2(m_sLogRootPath + "imuDistorted" + to_string(measures.lidar.frameId) + ".ply");
				fullOriginPly2.SetOnePatch(*feats_undistort);
				fullOriginPly2.Flush();
				fullOriginPly2.Close();
				m_vStatesGroup.push_back(state);
				SavePosPath(m_sLogRootPath + "_Framepos.txt");
				


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
					std::string plyPath = "E:\\data\\RTK\\RTK\\ProcessData\\";
					pcl::io::savePLYFile(plyPath + to_string(measures.lidar.frameId) + "outu.ply", *world_lidar);
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

						//计算每个点的协方差
						calcBodyCov(point_this, m_odoConfigPtr->s_dRangeCov, m_odoConfigPtr->s_dAngleCov, cov);

						point_this += Lidar_offset_to_IMU;
						M3D point_crossmat;
						point_crossmat << SKEW_SYM_MATRX(point_this);
						//世界坐标系点的协方差需要加上pos的不确定性
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
					LOG(INFO) << pv_list.size() << std::endl;
					buildVoxelMap(pv_list, m_odoConfigPtr->s_dVoxelSize, m_odoConfigPtr->s_iMaxLayer, m_odoConfigPtr->s_vLayerSize,
						m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_dPlannarThreshold,
						m_uVoxelMap);
					int tmp = 0;
					for (auto vm : m_uVoxelMap)
					{
						if (vm.second->temp_points_.size() == 0)
						{
							tmp++;
						}
					}
					SaveVoxelMap(measures.lidar.frameId);
					LOG(INFO) << "build voxel map" << std::endl;
					init_map = true;
					continue;
				}


				downSizeFilterSurf.setInputCloud(feats_undistort);
				downSizeFilterSurf.filter(*feats_down_body);
				sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
				PlyIo fullOriginPly1(m_sLogRootPath + "downSize" + to_string(measures.lidar.frameId)  + ".ply");
				fullOriginPly1.SetOnePatch(*feats_down_body);
				fullOriginPly1.Flush();
				fullOriginPly1.Close();
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
					body_var.push_back(cov);
				}
				
				std::cout << "Frame id: " << measures.lidar.frameId << std::endl;
				std::cout << "Pos: " << state.pos_end << std::endl;
				std::cout << "rot: " << state.rot_end << std::endl;
				//std::cout << "cov: " << state.cov << std::endl;

				for (int iterCount = 0; iterCount < m_odoConfigPtr->s_iMaxIteration; iterCount++)
				{
					std::vector<ptpl> ptpl_list;


					BaseCloudPtr world_lidar(new BaseCloud);
					transformLidar(state, m_pImu, feats_down_body, world_lidar);

					PlyIo fullOriginPly(m_sLogRootPath + "undistor_" + to_string(measures.lidar.frameId) + "_" + to_string(iterCount) + ".ply");
					fullOriginPly.SetOnePatch(*world_lidar);
					fullOriginPly.Flush();
					fullOriginPly.Close();

					vector<pointWithCov> pv_list;
					std::vector<M3D> var_list;
					for (size_t i = 0; i < feats_down_body->size(); i++) {
						pointWithCov pv;
						pv.point << feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z;
						pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
						M3D cov = body_var[i];
						M3D point_crossmat = crossmat_list[i];
						M3D rot_var = state.cov.block<3, 3>(0, 0);
						M3D t_var = state.cov.block<3, 3>(3, 3);
						M3D cov1 = state.rot_end * cov * state.rot_end.transpose()+ t_var;
						M3D cov2 = (-point_crossmat) * rot_var * (-point_crossmat.transpose());
						//cov = cov1 + cov2;
						pv.cov = cov;
						Eigen::Vector3d sigma_pv = pv.cov.diagonal();
						sigma_pv[0] = sqrt(sigma_pv[0]);
						sigma_pv[1] = sqrt(sigma_pv[1]);
						sigma_pv[2] = sqrt(sigma_pv[2]);
						pv_list.push_back(pv);
						var_list.push_back(cov);
					}

					std::vector<V3D> non_match_list;
					BuildResidualListOMP(m_uVoxelMap, m_odoConfigPtr->s_dVoxelSize, 3.0, m_odoConfigPtr->s_iMaxLayer, pv_list,
						ptpl_list, non_match_list);
					std::string pt_path = "E:\\data\\RTK\\RTK\\ProcessData\\";
					std::string pt_path1 = pt_path + to_string(measures.lidar.frameId) + "_" + to_string(iterCount) + "pt.txt";
					std::string pt_path2 = pt_path + to_string(measures.lidar.frameId) + "_" + to_string(iterCount) + "pl.txt";
					std::string pt_path3 = pt_path + to_string(measures.lidar.frameId) + "_" + to_string(iterCount) + "pw.txt";
					ofstream fd(pt_path1);
					ofstream fd1(pt_path2);
					ofstream fd2(pt_path3);
					

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
						
						
						Eigen::Vector3d curVec(pi_world.x - ptpl_list[i].center.x(), pi_world.y - ptpl_list[i].center.y(), pi_world.z - ptpl_list[i].center.z());
						float dis = curVec.dot(Eigen::Vector3d(pl.x, pl.y, pl.z));
						
						
						if (abs(dis) < 0.2)
						{
							effct_feat_num++;
							pl.intensity = dis;
							laserCloudOri->push_back(pi_body);
							corr_normvect->push_back(pl);
							total_residual += fabs(dis);
							
							fd << ptpl_list[i].point.x() << " " <<  ptpl_list[i].point.y() << " " <<  ptpl_list[i].point.z() << " " << std::endl;
							fd1 << ptpl_list[i].center.x() << " " << ptpl_list[i].center.y() << " " << ptpl_list[i].center.z() << std::endl;
							fd2 << pi_world.x << " " << pi_world.y << " " << pi_world.z << std::endl;
							
							
						}

						
					}
					fd.close();
					fd1.close();
					
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
						H_T_H.block<6, 6>(0, 0) = Hsub_T * Hsub;
						MD(DIM_STATE, DIM_STATE) && K_1 =
							(H_T_H + (state.cov).inverse()).inverse();
						K = K_1.block<DIM_STATE, 6>(0, 0) * Hsub_T;
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

						state = state_propagat + solution;

						rot_add = solution.block<3, 1>(0, 0);
						t_add = solution.block<3, 1>(3, 0);

						
						if ((rot_add.norm() * 57.3 - deltaR < 0.01) && (t_add.norm() * 100 -deltaT  < 0.015)) {
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
					std::cout << "Frame id: " << measures.lidar.frameId << " " << iterCount  << std::endl;
					std::cout << "Pos: " << "\t" << state.pos_end << std::endl;
					std::cout << "rot: " << "\t" << state.rot_end << std::endl;
					
					transformLidar(state, m_pImu, feats_down_body, world_lidar);
					PlyIo fullOriginPly3(m_sLogRootPath + "after_" + to_string(measures.lidar.frameId) + "_" +  to_string(iterCount) + ".ply");
					fullOriginPly3.SetOnePatch(*world_lidar);
					fullOriginPly3.Flush();
					fullOriginPly3.Close();

					//std::cout << "cov: " << state.cov << std::endl;
					if (EKF_stop_flg)
						break;
				}

				/*** add the  points to the voxel map ***/
				//continue;
				auto map_incremental_start = std::chrono::high_resolution_clock::now();
				BaseCloudPtr world_lidar(
					new BaseCloud);
				transformLidar(state, m_pImu, feats_down_body, world_lidar);
				PlyIo fullOriginPly(m_sLogRootPath + "after_" + to_string(measures.lidar.frameId) + ".ply");
				fullOriginPly.SetOnePatch(*world_lidar);
				fullOriginPly.Flush();
				fullOriginPly.Close();
				std::vector<pointWithCov> pv_list;
				for (size_t i = 0; i < world_lidar->size(); i++) {
					pointWithCov pv;
					pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
						world_lidar->points[i].z;
					M3D point_crossmat = crossmat_list[i];
					M3D cov = body_var[i];
					/*cov = state.rot_end * cov * state.rot_end.transpose() +
						(-point_crossmat) * state.cov.block<3, 3>(0, 0) *
						(-point_crossmat).transpose() +
						state.cov.block<3, 3>(3, 3);*/
					pv.cov = cov;
					pv_list.push_back(pv);
				}
				std::sort(pv_list.begin(), pv_list.end(), var_contrast);
				updateVoxelMap(pv_list, m_odoConfigPtr->s_dVoxelSize, m_odoConfigPtr->s_iMaxLayer, m_odoConfigPtr->s_vLayerSize,
					m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_iMaxPointsSize, m_odoConfigPtr->s_dPlannarThreshold,
					m_uVoxelMap);
				int tmp = 0;
				for (auto vm : m_uVoxelMap)
				{
					if (vm.second->temp_points_.size() == 0)
					{
						tmp++;
					}
				}
				std::cout << measures.lidar.frameId << " " << state.pos_end.x() << " " << state.pos_end.y() << " " << state.pos_end.z() << std::endl;
				std::cout << state.rot_end(0, 0) << " " << state.rot_end(0, 1) << " " << state.rot_end(0, 2) << " " << std::endl;
				std::cout << state.rot_end(1, 0) << " " << state.rot_end(1, 1) << " " << state.rot_end(1, 2) << " " << std::endl;
				std::cout << state.rot_end(2, 0) << " " << state.rot_end(2, 1) << " " << state.rot_end(2, 2) << std::endl;
				SaveVoxelMap(measures.lidar.frameId);
			}
		}

		return false;
	}

	bool VoxelMap::runR3live()
	{
		
		/*** variables definition ***/
		Eigen::Matrix< double, DIM_STATE, DIM_STATE > G, H_T_H, I_STATE;
		G.setZero();
		H_T_H.setZero();
		I_STATE.setIdentity();

		

		//BaseCloudPtr feats_undistort(new BaseCloud());
		BaseCloudPtr feats_down(new BaseCloud());
		//BaseCloudPtr laserCloudOri(new BaseCloud());
		BaseCloudPtr coeffSel(new BaseCloud());

		/*** variables initialize ***/
		double fov_deg = 360;
		double FOV_DEG, HALF_FOV_COS;
		FOV_DEG = fov_deg + 10;
		HALF_FOV_COS = std::cos((fov_deg + 10.0) * 0.5 * PI_M / 180.0);

		

		for (int i = 0; i < laserCloudNum; i++)
		{
			featsArray[i].reset(new BaseCloud());
		}

		MeasureGroup measures;
		//StatesGroup state, state_propagat;
		//------------------------------------------------------------------------------------------------------
		BaseCloudPtr feats_undistort(new BaseCloud());
		BaseCloudPtr feats_down_body(new BaseCloud());

		BaseCloudPtr normvec(new BaseCloud);
		BaseCloudPtr laserCloudOri(new BaseCloud);
		BaseCloudPtr laserCloudNoeffect(new BaseCloud);
		BaseCloudPtr corr_normvect(new BaseCloud);

		
		StatesGroup state, state_propagat;
		bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0, flg_map_initialized = 0, is_first_frame = true;
		double first_lidar_time = 0;
		bool init_map = false;
		double total_distance = 0;

		while (true)
		{
			
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			
			
			if (1)
			{
				// printf_line;
				
				if (sync_packages(measures) == 0)
				{
					continue;
				}
				int lidar_can_update = 1;
				
				
				
				m_pImu->Process(measures, state, feats_undistort);

				
				StatesGroup state_propagate(state);

				// cout << "G_lio_state.last_update_time =  " << std::setprecision(10) << g_lio_state.last_update_time -g_lidar_star_tim  << endl;
				
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
				
				/*** Compute the euler angle ***/
				Eigen::Vector3d euler_cur = RotMtoEuler(state.rot_end);
#ifdef DEBUG_PRINT
				std::cout << "current lidar time " << Measures.lidar_beg_time << " "
					<< "first lidar time " << frame_first_pt_time << std::endl;
				std::cout << "pre-integrated states: " << euler_cur.transpose() * 57.3 << " " << g_lio_state.pos_end.transpose() << " "
					<< g_lio_state.vel_end.transpose() << " " << g_lio_state.bias_g.transpose() << " " << g_lio_state.bias_a.transpose()
					<< std::endl;
#endif
				lasermap_fov_segment();
				downSizeFilterSurf.setInputCloud(feats_undistort);
				downSizeFilterSurf.filter(*feats_down);
				// cout <<"Preprocess cost time: " << tim.toc("Preprocess") << endl;
				/*** initialize the map kdtree ***/
				
				if ((feats_down->points.size() > 1) && (m_iKdtree.Root_Node == nullptr))
				{
					// std::vector<PointType> points_init = feats_down->points;
					m_iKdtree.set_downsample_param(m_odoConfigPtr->s_dFilterSizeMapMin);
					m_iKdtree.Build(feats_down->points);
					flg_map_initialized = true;
					continue;
				}

				if (m_iKdtree.Root_Node == nullptr)
				{
					flg_map_initialized = false;
					std::cout << "~~~~~~~ Initialize Map iKD-Tree Failed! ~~~~~~~" << std::endl;
					continue;
				}
				int featsFromMapNum = m_iKdtree.size();

				int feats_down_size = feats_down->points.size();

				/*** ICP and iterated Kalman filter update ***/
				BaseCloudPtr coeffSel_tmpt(new BaseCloud(*feats_down));
				BaseCloudPtr feats_down_updated(new BaseCloud(*feats_down));
				std::vector< double >     res_last(feats_down_size, 1000.0); // initial

				if (featsFromMapNum >= 5)
				{
					std::vector< bool >               point_selected_surf(feats_down_size, true);
					std::vector< std::vector< int > > pointSearchInd_surf(feats_down_size);
					std::vector< PointVector >        Nearest_Points(feats_down_size);

					int  rematch_num = 0;
					bool rematch_en = 0;
					flg_EKF_converged = 0;
					deltaR = 0.0;
					deltaT = 0.0;
					
					double maximum_pt_range = 0.0;
					// cout <<"Preprocess 2 cost time: " << tim.toc("Preprocess") << endl;
					for (int iterCount = 0; iterCount < m_odoConfigPtr->s_iMaxIteration; iterCount++)
					{
						
						laserCloudOri->clear();
						coeffSel->clear();

						/** closest surface search and residual computation **/
						for (int i = 0; i < feats_down_size; i += m_odoConfigPtr->s_iUpdatePointStep)
						{
							double     search_start = omp_get_wtime();
							PointType& pointOri_tmpt = feats_down->points[i];
							double     ori_pt_dis =
								sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z);
							maximum_pt_range = std::max(ori_pt_dis, maximum_pt_range);
							PointType& pointSel_tmpt = feats_down_updated->points[i];

							/* transform to world frame */
							pointBodyToWorld(state, &pointOri_tmpt, &pointSel_tmpt);
							std::vector< float > pointSearchSqDis_surf;

							auto& points_near = Nearest_Points[i];

							if (iterCount == 0 || rematch_en)
							{
								point_selected_surf[i] = true;
								/** Find the closest surfaces in the map **/
								m_iKdtree.Nearest_Search(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
								float max_distance = pointSearchSqDis_surf[NUM_MATCH_POINTS - 1];
								//  max_distance to add residuals
								// ANCHOR - Long range pt stragetry
								if (max_distance > m_odoConfigPtr->s_dMaximumKdtreeDis)
								{
									point_selected_surf[i] = false;
								}
							}

							if (point_selected_surf[i] == false)
								continue;

							// match_time += omp_get_wtime() - match_start;
							double pca_start = omp_get_wtime();
							/// PCA (using minimum square method)
							cv::Mat matA0(NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all(0));
							cv::Mat matB0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(-1));
							cv::Mat matX0(NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all(0));

							for (int j = 0; j < NUM_MATCH_POINTS; j++)
							{
								matA0.at< float >(j, 0) = points_near[j].x;
								matA0.at< float >(j, 1) = points_near[j].y;
								matA0.at< float >(j, 2) = points_near[j].z;
							}

							cv::solve(matA0, matB0, matX0, cv::DECOMP_QR); // TODO

							float pa = matX0.at< float >(0, 0);
							float pb = matX0.at< float >(1, 0);
							float pc = matX0.at< float >(2, 0);
							float pd = 1;

							float ps = sqrt(pa * pa + pb * pb + pc * pc);
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;

							bool planeValid = true;
							for (int j = 0; j < NUM_MATCH_POINTS; j++)
							{
								// ANCHOR -  Planar check
								if (fabs(pa * points_near[j].x + pb * points_near[j].y + pc * points_near[j].z + pd) >
									m_odoConfigPtr->s_dPlanarCheckDis) // Raw 0.05
								{
									// ANCHOR - Far distance pt processing
									if (ori_pt_dis < maximum_pt_range * 0.90 || (ori_pt_dis < m_odoConfigPtr->s_dRangDis))
										// if(1)
									{
										planeValid = false;
										point_selected_surf[i] = false;
										break;
									}
								}
							}

							if (planeValid)
							{
								float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
								float s = 1 - 0.9 * fabs(pd2) /
									sqrt(sqrt(pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y +
										pointSel_tmpt.z * pointSel_tmpt.z));
								// ANCHOR -  Point to plane distance
								double acc_distance = (ori_pt_dis < m_odoConfigPtr->s_dRangDis) ? m_odoConfigPtr->s_dMaximumResDis : 1.0;
								if (pd2 < acc_distance)
								{
									// if(std::abs(pd2) > 5 * res_mean_last)
									// {
									//     point_selected_surf[i] = false;
									//     res_last[i] = 0.0;
									//     continue;
									// }
									point_selected_surf[i] = true;
									coeffSel_tmpt->points[i].x = pa;
									coeffSel_tmpt->points[i].y = pb;
									coeffSel_tmpt->points[i].z = pc;
									coeffSel_tmpt->points[i].intensity = pd2;
									res_last[i] = std::abs(pd2);
								}
								else
								{
									point_selected_surf[i] = false;
								}
							}
							
						}
						
						double total_residual = 0.0;
						int laserCloudSelNum = 0;

						for (int i = 0; i < coeffSel_tmpt->points.size(); i++)
						{
							if (point_selected_surf[i] && (res_last[i] <= 2.0))
							{
								laserCloudOri->push_back(feats_down->points[i]);
								coeffSel->push_back(coeffSel_tmpt->points[i]);
								total_residual += res_last[i];
								laserCloudSelNum++;
							}
						}
						double res_mean_last = total_residual / laserCloudSelNum;

						

						/*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
						Eigen::MatrixXd Hsub(laserCloudSelNum, 6);
						Eigen::VectorXd meas_vec(laserCloudSelNum);
						Hsub.setZero();

						for (int i = 0; i < laserCloudSelNum; i++)
						{
							const PointType& laser_p = laserCloudOri->points[i];
							Eigen::Vector3d  point_this(laser_p.x, laser_p.y, laser_p.z);
							point_this += Lidar_offset_to_IMU;
							Eigen::Matrix3d point_crossmat;
							point_crossmat << SKEW_SYM_MATRX(point_this);

							/*** get the normal vector of closest surface/corner ***/
							const PointType& norm_p = coeffSel->points[i];
							Eigen::Vector3d  norm_vec(norm_p.x, norm_p.y, norm_p.z);

							/*** calculate the Measuremnt Jacobian matrix H ***/
							Eigen::Vector3d A(point_crossmat * g_lio_state.rot_end.transpose() * norm_vec);
							Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

							/*** Measuremnt: distance to the closest surface/corner ***/
							meas_vec(i) = -norm_p.intensity;
						}

						Eigen::Vector3d                           rot_add, t_add, v_add, bg_add, ba_add, g_add;
						Eigen::Matrix< double, DIM_STATE, 1 > solution;
						Eigen::MatrixXd                           K(DIM_STATE, laserCloudSelNum);

						/*** Iterative Kalman Filter Update ***/
						if (!flg_EKF_inited)
						{
							cout << "Run EKF init"  << endl;
							/*** only run in initialization period ***/
							//set_initial_state_cov(g_lio_state);
						}
						else
						{
							// cout << ANSI_COLOR_RED_BOLD << "Run EKF uph" << ANSI_COLOR_RESET << endl;
							auto&& Hsub_T = Hsub.transpose();
							H_T_H.block< 6, 6 >(0, 0) = Hsub_T * Hsub;
							Eigen::Matrix< double, DIM_STATE, DIM_STATE >&& K_1 =
								(H_T_H + (g_lio_state.cov / LASER_POINT_COV).inverse()).inverse();
							K = K_1.block< DIM_STATE, 6 >(0, 0) * Hsub_T;

							auto vec = state_propagate - g_lio_state;
							solution = K * (meas_vec - Hsub * vec.block< 6, 1 >(0, 0));
							// double speed_delta = solution.block( 0, 6, 3, 1 ).norm();
							// if(solution.block( 0, 6, 3, 1 ).norm() > 0.05 )
							// {
							//     solution.block( 0, 6, 3, 1 ) = solution.block( 0, 6, 3, 1 ) / speed_delta * 0.05;
							// }

							g_lio_state = state_propagate + solution;
							
							// cout << ANSI_COLOR_RED_BOLD << "Run EKF uph, vec = " << vec.head<9>().transpose() << ANSI_COLOR_RESET << endl;
							rot_add = solution.block< 3, 1 >(0, 0);
							t_add = solution.block< 3, 1 >(3, 0);
							flg_EKF_converged = false;
							if (((rot_add.norm() * 57.3 - deltaR) < 0.01) && ((t_add.norm() * 100 - deltaT) < 0.015))
							{
								flg_EKF_converged = true;
							}

							deltaR = rot_add.norm() * 57.3;
							deltaT = t_add.norm() * 100;
						}

						// printf_line;
						state.last_update_time = measures.lidar.endTime;
						euler_cur = RotMtoEuler(state.rot_end);
						

						/*** Rematch Judgement ***/
						rematch_en = false;
						if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (m_odoConfigPtr->s_iMaxIteration - 2))))
						{
							rematch_en = true;
							rematch_num++;
						}

						/*** Convergence Judgements and Covariance Update ***/
						// if (rematch_num >= 10 || (iterCount == NUM_MAX_ITERATIONS - 1))
						if (rematch_num >= 2 || (iterCount == m_odoConfigPtr->s_iMaxIteration - 1)) // Fast lio ori version.
						{
							if (flg_EKF_inited)
							{
								/*** Covariance Update ***/
								G.block< DIM_STATE, 6 >(0, 0) = K * Hsub;
								g_lio_state.cov = (I_STATE - G) * g_lio_state.cov;
								total_distance += (g_lio_state.pos_end - position_last).norm();
								position_last = g_lio_state.pos_end;

								// std::cout << "position: " << g_lio_state.pos_end.transpose() << " total distance: " << total_distance << std::endl;
							}
							
							break;
						}
						
						// cout << "Match cost time: " << match_time * 1000.0
						//      << ", search cost time: " << kdtree_search_time*1000.0
						//      << ", PCA cost time: " << pca_time*1000.0
						//      << ", solver_cost: " << solve_time * 1000.0 << endl;
						// cout <<"Iter cost time: " << tim.toc("Iter") << endl;
					}

					

					/*** add new frame points to map ikdtree ***/
					PointVector points_history;
					m_iKdtree.acquire_removed_points(points_history);

					memset(cube_updated, 0, sizeof(cube_updated));

					for (int i = 0; i < points_history.size(); i++)
					{
						PointType& pointSel = points_history[i];

						int cubeI = int((pointSel.x + 0.5 * m_odoConfigPtr->s_dCubeLen) / m_odoConfigPtr->s_dCubeLen) + laserCloudCenWidth;
						int cubeJ = int((pointSel.y + 0.5 * m_odoConfigPtr->s_dCubeLen) / m_odoConfigPtr->s_dCubeLen) + laserCloudCenHeight;
						int cubeK = int((pointSel.z + 0.5 * m_odoConfigPtr->s_dCubeLen) / m_odoConfigPtr->s_dCubeLen) + laserCloudCenDepth;

						if (pointSel.x + 0.5 * m_odoConfigPtr->s_dCubeLen < 0)
							cubeI--;		   
						if (pointSel.y + 0.5 * m_odoConfigPtr->s_dCubeLen < 0)
							cubeJ--;		   
						if (pointSel.z + 0.5 * m_odoConfigPtr->s_dCubeLen < 0)
							cubeK--;

						if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth)
						{
							int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
							featsArray[cubeInd]->push_back(pointSel);
						}
					}

					for (int i = 0; i < feats_down_size; i++)
					{
						/* transform to world frame */
						pointBodyToWorld(state, &(feats_down->points[i]), &(feats_down_updated->points[i]));
					}
					

					m_iKdtree.Add_Points(feats_down_updated->points, true);

					
				}
			}
			
		}
		return 0;
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
	bool VoxelMap::JugeStatic(MeasureGroup &meas)
	{
		int iNum = 0;
		
		V3D vMeanAcc, vCurAcc, cov_acc;
		vMeanAcc << double(meas.imu.front().acc[0]), double(meas.imu.front().acc[1]), double(meas.imu.front().acc[2]);
		cov_acc << 0, 0, 0;
		for (auto &aImu : meas.imu)
		{
			iNum++;
			vCurAcc << float(aImu.acc[0]), aImu.acc[1], aImu.acc[2];
			vMeanAcc += (vCurAcc - vMeanAcc) / iNum;
			cov_acc = cov_acc * (iNum - 1.0) / iNum +
				(vCurAcc - vMeanAcc).cwiseProduct(vCurAcc - vMeanAcc) *
				(iNum - 1.0) / (iNum * iNum);

		}
		if (cov_acc.x() > m_odoConfigPtr->s_dStaticParam && cov_acc.y() > m_odoConfigPtr->s_dStaticParam && cov_acc.z() > m_odoConfigPtr->s_dStaticParam)
		{
			meas.isStatic = false;
		}
		else
			meas.isStatic = true;
		return true;
	}
	void VoxelMap::SaveVoxelMap(int id)
	{
		std::string plyPath = "E:\\data\\RTK\\RTK\\ProcessData\\";
		std::vector<Plane> pub_plane_list;
		/*ColorCloudPtr colorCloud(new ColorCloud);
		ColorCloudPtr colorCloudPlane(new ColorCloud);*/
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloudPlane(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		ColorPoint colorPoint;
		for (auto iter = m_uVoxelMap.begin(); iter != m_uVoxelMap.end(); iter++) {
			GetVoxelMap(iter->second, colorCloud, colorCloudPlane);
		}
		pcl::io::savePLYFile(plyPath + "voxelmap_" + to_string(id) + ".ply", *colorCloud);
		pcl::io::savePLYFile(plyPath + "voxelmap_" + to_string(id) + "plane.ply", *colorCloudPlane);
			
	}
	void VoxelMap::GetVoxelMap(const OctoTree* current_octo, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorPointCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloudPlane)
	{
		if (current_octo == nullptr)
		{
			return;
		}
		if (current_octo->octo_state_ == 0)
		{
			//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorPoint
			pcl::PointXYZRGBNormal colorPoint;
			bool isPlane = -1;
			if (current_octo->plane_ptr_->is_plane)
				isPlane = 1;
			if (current_octo->temp_points_.size() != 0)
			{

				int r = (rand() % 255 + 10) % 255;
				int g = (rand() % 255 + 10) % 255;
				int b = (rand() % 255 + 10) % 255;
				for (auto& pt : current_octo->temp_points_)
				{
					colorPoint.x = pt.point[0];
					colorPoint.y = pt.point[1];
					colorPoint.z = pt.point[2];
					colorPoint.r = r;
					colorPoint.g = g;
					colorPoint.b = b;
					colorPoint.normal_x = current_octo->plane_ptr_->min_eigen_value;
					colorPoint.normal_y = current_octo->plane_ptr_->mid_eigen_value;
					colorPoint.normal_z = current_octo->plane_ptr_->max_eigen_value;
					colorPointCloud->points.push_back(colorPoint);
					//if (isPlane)
					if (current_octo->plane_ptr_->min_eigen_value < 0.05 && current_octo->plane_ptr_->d < 0.4)
					{
						colorPoint.normal_x = current_octo->plane_ptr_->min_eigen_value;
						colorPoint.normal_y = current_octo->plane_ptr_->mid_eigen_value;
						colorPoint.normal_z = current_octo->plane_ptr_->max_eigen_value;
						colorCloudPlane->points.push_back(colorPoint);
					}
				}
			}
			return;
		}
		else {
			for (int i = 0; i < 8; i++)
			{
				GetVoxelMap(current_octo->leaves_[i], colorPointCloud, colorCloudPlane);
			}
		}
		




	}
	void VoxelMap::SavePosPath(std::string pospath)
	{
		if (!m_vStatesGroup.empty())
		{
			ofstream fd(pospath/*, std::ios::out || std::ios::app*/);
			for (auto sGroup : m_vStatesGroup)
				fd << sGroup.pos_end.x() << " " << sGroup.pos_end.y() << " " << sGroup.pos_end.z() << " " <<  RotMtoEuler(sGroup.rot_end)[0] * 57.3 << " " << RotMtoEuler(sGroup.rot_end)[1] * 57.3 <<  " " << RotMtoEuler(sGroup.rot_end)[2] * 57.3 << std::endl;
			fd.close();
		}
	}
	void VoxelMap::lasermap_fov_segment()
	{
		laserCloudValidNum = 0;
		pointBodyToWorld(g_lio_state, XAxisPoint_body, XAxisPoint_world);
		int centerCubeI = int((g_lio_state.pos_end(0) + 0.5 * m_odoConfigPtr->s_dCubeLen) / m_odoConfigPtr->s_dCubeLen) + laserCloudCenWidth;
		int centerCubeJ = int((g_lio_state.pos_end(1) + 0.5 * m_odoConfigPtr->s_dCubeLen) / m_odoConfigPtr->s_dCubeLen) + laserCloudCenHeight;
		int centerCubeK = int((g_lio_state.pos_end(2) + 0.5 * m_odoConfigPtr->s_dCubeLen) / m_odoConfigPtr->s_dCubeLen) + laserCloudCenDepth;
		if (g_lio_state.pos_end(0) + 0.5 * m_odoConfigPtr->s_dCubeLen < 0)
			centerCubeI--;				   
		if (g_lio_state.pos_end(1) + 0.5 * m_odoConfigPtr->s_dCubeLen < 0)
			centerCubeJ--;				   
		if (g_lio_state.pos_end(2) + 0.5 * m_odoConfigPtr->s_dCubeLen < 0)
			centerCubeK--;
		bool last_inFOV_flag = 0;
		int  cube_index = 0;
		cub_needrm.clear();
		cub_needad.clear();
		
		double t_begin = omp_get_wtime();

		while (centerCubeI < FOV_RANGE + 1)
		{
			for (int j = 0; j < laserCloudHeight; j++)
			{
				for (int k = 0; k < laserCloudDepth; k++)
				{
					int i = laserCloudWidth - 1;

					BaseCloudPtr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
					last_inFOV_flag = _last_inFOV[cube_index];

					for (; i >= 1; i--)
					{
						featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i - 1, j, k)];
						_last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i - 1, j, k)];
					}

					featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
					_last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
					laserCloudCubeSurfPointer->clear();
				}
			}
			centerCubeI++;
			laserCloudCenWidth++;
		}

		while (centerCubeI >= laserCloudWidth - (FOV_RANGE + 1))
		{
			for (int j = 0; j < laserCloudHeight; j++)
			{
				for (int k = 0; k < laserCloudDepth; k++)
				{
					int i = 0;

					BaseCloudPtr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
					last_inFOV_flag = _last_inFOV[cube_index];

					for (; i >= 1; i--)
					{
						featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i + 1, j, k)];
						_last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i + 1, j, k)];
					}

					featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
					_last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeI--;
			laserCloudCenWidth--;
		}

		while (centerCubeJ < (FOV_RANGE + 1))
		{
			for (int i = 0; i < laserCloudWidth; i++)
			{
				for (int k = 0; k < laserCloudDepth; k++)
				{
					int j = laserCloudHeight - 1;

					BaseCloudPtr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
					last_inFOV_flag = _last_inFOV[cube_index];

					for (; i >= 1; i--)
					{
						featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j - 1, k)];
						_last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j - 1, k)];
					}

					featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
					_last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeJ++;
			laserCloudCenHeight++;
		}

		while (centerCubeJ >= laserCloudHeight - (FOV_RANGE + 1))
		{
			for (int i = 0; i < laserCloudWidth; i++)
			{
				for (int k = 0; k < laserCloudDepth; k++)
				{
					int                       j = 0;
					BaseCloudPtr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
					last_inFOV_flag = _last_inFOV[cube_index];

					for (; i >= 1; i--)
					{
						featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j + 1, k)];
						_last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j + 1, k)];
					}

					featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
					_last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeJ--;
			laserCloudCenHeight--;
		}

		while (centerCubeK < (FOV_RANGE + 1))
		{
			for (int i = 0; i < laserCloudWidth; i++)
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					int                       k = laserCloudDepth - 1;
					BaseCloudPtr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
					last_inFOV_flag = _last_inFOV[cube_index];

					for (; i >= 1; i--)
					{
						featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j, k - 1)];
						_last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j, k - 1)];
					}

					featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
					_last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
					laserCloudCubeSurfPointer->clear();
				}
			}

			centerCubeK++;
			laserCloudCenDepth++;
		}

		while (centerCubeK >= laserCloudDepth - (FOV_RANGE + 1))
		{
			for (int i = 0; i < laserCloudWidth; i++)
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					int                       k = 0;
					BaseCloudPtr laserCloudCubeSurfPointer = featsArray[get_cube_index(i, j, k)];
					last_inFOV_flag = _last_inFOV[cube_index];

					for (; i >= 1; i--)
					{
						featsArray[get_cube_index(i, j, k)] = featsArray[get_cube_index(i, j, k + 1)];
						_last_inFOV[get_cube_index(i, j, k)] = _last_inFOV[get_cube_index(i, j, k + 1)];
					}

					featsArray[get_cube_index(i, j, k)] = laserCloudCubeSurfPointer;
					_last_inFOV[get_cube_index(i, j, k)] = last_inFOV_flag;
					laserCloudCubeSurfPointer->clear();
				}
			}
			centerCubeK--;
			laserCloudCenDepth--;
		}

		cube_points_add->clear();
		featsFromMap->clear();
		memset(now_inFOV, 0, sizeof(now_inFOV));
		
		
#ifdef USE_ikdtree
		if (cub_needrm.size() > 0)
			m_iKdtree.Delete_Point_Boxes(cub_needrm);
		
		// s_plot4.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
		if (cub_needad.size() > 0)
			m_iKdtree.Add_Point_Boxes(cub_needad);
		
		// s_plot5.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
		if (cube_points_add->points.size() > 0)
			m_iKdtree.Add_Points(cube_points_add->points, true);
#endif
		// s_plot6.push_back(omp_get_wtime() - t_begin);
	}

}