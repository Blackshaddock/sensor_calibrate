#include "r3live_lio.h"

int inputID = 0;

int imuNum = 0;
int odomNum = 0;
int laserNum = 0;


bool R3LIVE::sync_packages(MeasureGroup& meas, int num)
{
    if (m_dLidarBuffer.empty() || m_dImuBuffer.empty())
    {
        return false;
    }

    /*** push lidar frame ***/
    if (!m_bLidarPushed)
    {
        meas.lidar = m_dLidarBuffer.front();
        if (meas.lidar.cloudPtr->points.size() <= 1)
        {
            return false;
        }
        m_bLidarPushed = true;

    }

    if (m_dLastImuTime < meas.lidar.endTime)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return false;
    }

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
        m_mImumutex.lock();
        m_dImuBuffer.pop_front();
        m_mImumutex.unlock();
    }
    m_bLidarPushed = false;
    m_mLidarmutex.lock();
    m_dLidarBuffer.pop_front();
    m_mLidarmutex.unlock();
    return 1;
}

// project lidar frame to world
void R3LIVE::pointBodyToWorld(PointType const* const pi, PointType* const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);

    Eigen::Vector3d p_global(g_lio_state.rot_end * (/*m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_eRL2e**/p_body /*+ m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_eTL2e*/)+g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void R3LIVE::RGBpointBodyToWorld(PointType const* const pi, BasePoint* const po)
{
    Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body)+g_lio_state.pos_end);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity * 10000;
}

int R3LIVE::get_cube_index(const int& i, const int& j, const int& k)
{
    return (i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
}

bool R3LIVE::center_in_FOV(Eigen::Vector3f cube_p)
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast< float >() - cube_p;
    float           squaredSide1 = dis_vec.transpose() * dis_vec;

    if (squaredSide1 < 0.4 * cube_len * cube_len)
        return true;

    dis_vec = XAxisPoint_world.cast< float >() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos =
        fabs(squaredSide1 <= 3) ? 1.0 : (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));

    return ((ang_cos > HALF_FOV_COS) ? true : false);
}

bool R3LIVE::if_corner_in_FOV(Eigen::Vector3f cube_p)
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast< float >() - cube_p;
    float           squaredSide1 = dis_vec.transpose() * dis_vec;
    dis_vec = XAxisPoint_world.cast< float >() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;
    float ang_cos =
        fabs(squaredSide1 <= 3) ? 1.0 : (LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2) / (2 * LIDAR_SP_LEN * sqrt(squaredSide1));
    return ((ang_cos > HALF_FOV_COS) ? true : false);
}

void R3LIVE::lasermap_fov_segment()
{
    LOG(INFO) << cube_len << std::endl;
    laserCloudValidNum = 0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    int centerCubeI = int((g_lio_state.pos_end(0) + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
    int centerCubeJ = int((g_lio_state.pos_end(1) + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
    int centerCubeK = int((g_lio_state.pos_end(2) + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;
    if (g_lio_state.pos_end(0) + 0.5 * cube_len < 0)
        centerCubeI--;
    if (g_lio_state.pos_end(1) + 0.5 * cube_len < 0)
        centerCubeJ--;
    if (g_lio_state.pos_end(2) + 0.5 * cube_len < 0)
        centerCubeK--;
    bool last_inFOV_flag = 0;
    int  cube_index = 0;
    cub_needrm.clear();
    cub_needad.clear();
    double t_begin = omp_get_wtime();

    while (centerCubeI < m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFovRange + 1)
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

    while (centerCubeI >= laserCloudWidth - (m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFovRange + 1))
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

    while (centerCubeJ < (m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFovRange + 1))
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

    while (centerCubeJ >= laserCloudHeight - (m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFovRange + 1))
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

    while (centerCubeK < (m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFovRange + 1))
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

    while (centerCubeK >= laserCloudDepth - (m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFovRange + 1))
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
    copy_time = omp_get_wtime() - t_begin;
    double fov_check_begin = omp_get_wtime();

    fov_check_time = omp_get_wtime() - fov_check_begin;

    double readd_begin = omp_get_wtime();
#ifdef USE_ikdtree
    if (cub_needrm.size() > 0)
        ikdtree.Delete_Point_Boxes(cub_needrm);
    delete_box_time = omp_get_wtime() - readd_begin;
    // s_plot4.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if (cub_needad.size() > 0)
        ikdtree.Add_Point_Boxes(cub_needad);
    readd_box_time = omp_get_wtime() - readd_begin - delete_box_time;
    // s_plot5.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if (cube_points_add->points.size() > 0)
        ikdtree.Add_Points(cube_points_add->points, true);
#endif
    readd_time = omp_get_wtime() - readd_begin - delete_box_time - readd_box_time;
    // s_plot6.push_back(omp_get_wtime() - t_begin);
}

fstream fdsweepdata;

int mergeCount = 0;

vector<sensor_msgs::PointCloud2::Ptr> msgvec;

ofstream fdsweep;
double lastlasertime = -1;





void R3LIVE::set_initial_state_cov(StatesGroup& state)
{
    // Set cov

    state.cov = state.cov.setIdentity() * INIT_COV;
    // state.cov.block(18, 18, 6 , 6 ) = state.cov.block(18, 18, 6 , 6 ) .setIdentity() * 0.1;
    // state.cov.block(24, 24, 5 , 5 ) = state.cov.block(24, 24, 5 , 5 ).setIdentity() * 0.001;
    state.cov.block(0, 0, 3, 3) = M3D::Identity() * 1e-5;   // R
    state.cov.block(3, 3, 3, 3) = M3D::Identity() * 1e-5;   // T
    state.cov.block(6, 6, 3, 3) = M3D::Identity() * 1e-5;   // vel
    state.cov.block(9, 9, 3, 3) = M3D::Identity() * 1e-3;   // bias_g
    state.cov.block(12, 12, 3, 3) = M3D::Identity() * 1e-1; // bias_a
    state.cov.block(15, 15, 3, 3) = M3D::Identity() * 1e-5; // Gravity
    //state.cov( 24, 24 ) = 0.00001;
    //state.cov.block( 18, 18, 6, 6 ) = state.cov.block( 18, 18, 6, 6 ).setIdentity() *  1e-3; // Extrinsic between camera and IMU.
    //state.cov.block( 25, 25, 4, 4 ) = state.cov.block( 25, 25, 4, 4 ).setIdentity() *  1e-3; // Camera intrinsic.
}


int R3LIVE::service_LIO_update()
{
    m_sLogRootPath = "E:\\data\\RTK\\RTK\\ProcessData\\";
    m_sOptimRootPath = "E:\\data\\RTK\\RTK\\ProcessData\\Optim\\";
    m_sMatchRootPath = "E:\\data\\RTK\\RTK\\ProcessData\\Match\\";
    /*** variables definition ***/
    Eigen::Matrix< double, DIM_STATE, DIM_STATE > G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    BaseCloudPtr init_points(new BaseCloud());
    BaseCloudPtr feats_undistort(new BaseCloud());
    BaseCloudPtr feats_down(new BaseCloud());
    BaseCloudPtr laserCloudOri(new BaseCloud());
    BaseCloudPtr coeffSel(new BaseCloud());


    
    /*** variables initialize ***/
    FOV_DEG = fov_deg + 10;
    HALF_FOV_COS = std::cos((fov_deg + 10.0) * 0.5 * PI_M / 180.0);

    for (int i = 0; i < laserCloudNum; i++)
    {
        featsArray[i].reset(new BaseCloud());
    }
    Eigen::Matrix4d rotationToGravaty = Eigen::Matrix4d::Identity();
    int rotAccumulateNum = 0;
    std::shared_ptr< ImuProcess > p_imu(new ImuProcess(cov_omg_n, cov_acc_n, cov_gry_n, cov_bias_acc_n, cov_bias_gry_n));
    m_imu_process = p_imu;
    int odomMaxAccumulateNum = 20;
    //------------------------------------------------------------------------------------------------------
    set_initial_state_cov(g_lio_state);
    int frame_id = 0;
    ofstream fdpos;
    if (plyer)
        fdpos.open(m_map_output_dir + "//pos.xyz");

    while (true)
    {
        if (flg_exit)
            break;
        if (m_dImuBuffer.empty() || m_dLidarBuffer.empty())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(8));
            continue;
        }
        if (1)
        {

            if (sync_packages(Measures, 0) == 0)
            {
                continue;
            }
            int lidar_can_update = 1;
           // bool bStatic = JugeStatic(Measures);
            if(p_imu->imu_need_init_)
                p_imu->Process(Measures, g_lio_state, feats_undistort);
            StatesGroup state_propagate(g_lio_state);
            if (Measures.lidar.frameId > 40)
            {
                PlyIo fullOriginPly02(m_sLogRootPath + "imuDistortedNo" + to_string(Measures.lidar.frameId) + ".ply");
                fullOriginPly02.SetOnePatch(*Measures.lidar.cloudPtr);
                fullOriginPly02.Flush();
                fullOriginPly02.Close();
                p_imu->Process(Measures, g_lio_state, feats_undistort);
                PlyIo fullOriginPly03(m_sLogRootPath + "imuDistorted1No" + to_string(Measures.lidar.frameId) + ".ply");
                fullOriginPly03.SetOnePatch(*feats_undistort);
                fullOriginPly03.Flush();
                fullOriginPly03.Close();
            }
            

            if (Measures.lidar.frameId < 10)
            {
                *init_points += *Measures.lidar.cloudPtr;
                continue;
            }
            if (Measures.lidar.frameId == 10)
            {
                *init_points += *Measures.lidar.cloudPtr;
               // feats_undistort = init_points;
                
                transformLidar(init_points, feats_undistort, 1);
                
            }
            
            // cout << "G_lio_state.last_update_time =  " << std::setprecision(10) << g_lio_state.last_update_time -g_lidar_star_tim  << endl;
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                frame_first_pt_time = Measures.lidar.startTime;
                std::cout << "not ready for odometry" << std::endl;
                continue;
            }

            if ((Measures.lidar.startTime - frame_first_pt_time) < INIT_TIME)
            {
                flg_EKF_inited = false;
                std::cout << "||||||||||Initiallizing LiDAR||||||||||" << std::endl;
            }
            else
            {
                flg_EKF_inited = true;
            }
           
           
                
            /*** Compute the euler angle ***/
            Eigen::Vector3d euler_cur = RotMtoEuler(g_lio_state.rot_end);
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
            PlyIo fullOriginPly2(m_sLogRootPath + "imuDistorted" + to_string(Measures.lidar.frameId) + ".ply");
            fullOriginPly2.SetOnePatch(*feats_undistort);
            fullOriginPly2.Flush();
            fullOriginPly2.Close();
            transformLidar(Measures.lidar.cloudPtr, m_bUpdateBaseCloud, 1);
            PlyIo fullOriginPly5(m_sLogRootPath + "BeforeimuDistorted" + to_string(Measures.lidar.frameId) + ".ply");
            fullOriginPly5.SetOnePatch(*m_bUpdateBaseCloud);
            fullOriginPly5.Flush();
            fullOriginPly5.Close();
            // cout <<"Preprocess cost time: " << tim.toc("Preprocess") << endl;
            /*** initialize the map kdtree ***/
//#ifdef UseVoxelMap
            std::vector<pointWithCov> pv_list;
            pointWithCov pv;
            if ((feats_down->points.size() > 1) && (flg_map_initialized == false))
            {
                for (auto pt : feats_down->points)
                {
                    pv.position << pt.x, pt.y, pt.z;
                    pv.point = pt;
                    pv_list.push_back(pv);
                }
                buildVoxelMap(pv_list, m_voxelOptionsCfgPtr->s_dVoxelSize, 
                    m_uVoxelMap);
                SaveVoxelMap(Measures.lidar.frameId);
                flg_map_initialized = true;
                continue;
            }
//#endif // UseVoxelMap

            //if ((feats_down->points.size() > 1) && (ikdtree.Root_Node == nullptr))
            //{
            //    // std::vector<PointType> points_init = feats_down->points;
            //    ikdtree.set_downsample_param(filter_size_map_min);
            //    ikdtree.Build(feats_down->points);
            //    flg_map_initialized = true;
            //    continue;
            //}





            double maxdis = 0;
            for (int i = 0; i < feats_down->size(); i++)
            {
                double dis = sqrt(feats_down->points[i].x * feats_down->points[i].x + feats_down->points[i].y * feats_down->points[i].y + feats_down->points[i].z * feats_down->points[i].z);
                if (dis > maxdis)
                    maxdis = dis;
            }

            if (m_indoor_downsample_dis > 0 && maxdis < m_indoor_downsample_dis)
            {
                downSizeFilterSurfMin.setInputCloud(feats_undistort);
                downSizeFilterSurfMin.filter(*feats_down);
            }


           /* if (ikdtree.Root_Node == nullptr)
            {
                flg_map_initialized = false;
                std::cout << "~~~~~~~ Initialize Map iKD-Tree Failed! ~~~~~~~" << std::endl;
                continue;
            }*/
            


            if (abs(inputID - Measures.lidar.frameId) > 100)
            {
                if (feats_down->points.size() > m_max_feature_points)
                {
                    double maxpoint = double(m_max_feature_points);

                    //	maxpoint = m_max_feature_points - (double(abs(inputID - Measures.rangeid)) - 50.);

                    double rateFilter = double(feats_down->size()) / maxpoint;
                    BaseCloud feats_down_temp;

                    for (double i = 0; i < feats_down->size(); i += rateFilter)
                    {
                        int curIdx = floor(i);
                        feats_down_temp.push_back(feats_down->points[i]);
                    }
                    feats_down->clear();
                    *feats_down = feats_down_temp;

                }
            }


            int feats_down_size = feats_down->points.size();

            /*** ICP and iterated Kalman filter update ***/
            BaseCloudPtr coeffSel_tmpt(new BaseCloud(*feats_down));
            BaseCloudPtr feats_down_updated(new BaseCloud(*feats_down));
            std::vector< double >     res_last(feats_down_size, 1000.0); // initial
            //int pointnum = 0;
            int curIterNum = 0;

            int pointsetp = m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iPointStep;

            if (feats_down_size < m_min_feature_points)
            {
                pointsetp = 1;
            }



            if (true)
            {
                std::vector< bool >               point_selected_surf(feats_down_size, true);
                std::vector< std::vector< int > > pointSearchInd_surf(feats_down_size);
                std::vector<BaseCloud>            Nearest_Points(feats_down_size);
                //std::vector< PointVector >        Nearest_Points(feats_down_size);
                int  rematch_num = 0;
                bool rematch_en = 0;
                flg_EKF_converged = 0;
                deltaR = 0.0;
                deltaT = 0.0;

                double maximum_pt_range = 0.0;

                //使用gicp做预处理


                // cout <<"Preprocess 2 cost time: " << tim.toc("Preprocess") << endl;
                for (int iterCount = 0; iterCount < m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iMaxIteration; iterCount++)
                {
                    laserCloudOri->clear();
                    coeffSel->clear();
                    curIterNum++;

                    /** closest surface search and residual computation **/
                    for (int i = 0; i < feats_down_size; i += pointsetp)
                    {
                        double     search_start = omp_get_wtime();
                        PointType& pointOri_tmpt = feats_down->points[i];
                        double     ori_pt_dis = sqrt(pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z);
                        maximum_pt_range = std::max(ori_pt_dis, maximum_pt_range);
                        PointType& pointSel_tmpt = feats_down_updated->points[i];
                        pointBodyToWorld(&pointOri_tmpt, &pointSel_tmpt);
                        //找到该点周围的voxel,并将voxel的点构建kdtree
                        std::vector< float > pointSearchSqDis_surf;
                        auto& points_near = Nearest_Points[i];

                        if (iterCount < 5 /*|| rematch_en*/)
                        {

                            point_selected_surf[i] = true;
                            /** Find the closest surfaces in the map **/
                            if (!NearestSearchInVoxel(pointSel_tmpt, m_voxelOptionsCfgPtr->s_dVoxelSize, m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iNumMatchPoints, points_near, pointSearchSqDis_surf))
                            {
                                point_selected_surf[i] = false;
                                continue;
                            }
                            
                            //ikdtree.Nearest_Search(pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf);
                            float max_distance = pointSearchSqDis_surf.back();
                            //  max_distance to add residuals
                            // ANCHOR - Long range pt stragetry
                            if (max_distance > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dMaxmiumKdtreeDis)
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
                                m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dPlanarCheckDis) // Raw 0.05
                            {
                                // ANCHOR - Far distance pt processing
                                if (ori_pt_dis < maximum_pt_range * 0.90 || (ori_pt_dis < m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dMaxmiumResDis))
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
                            double acc_distance = (ori_pt_dis < m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dRangPtDis) ?
                                m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dMaxmiumResDis : 1.0;
                            if (pd2 < acc_distance)
                            {
                                // if(std::abs(pd2) > 5 * res_mean_last)
                                // {
                                //     point_selected_surf[i] = false;
                                //     res_last[i] = 0.0;
                                //     continue;
                                // }
                                        //pointnum++;
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

                        //{
                        //	cout << "feature num: " << pointnum  << endl;	
                        //	break;
                        //}
                    }

                    double total_residual = 0.0;
                    laserCloudSelNum = 0;
                    std::string sMatchPath = m_sMatchRootPath + "Match_" + to_string(Measures.lidar.frameId) + "_" + to_string(iterCount) + ".txt";
                    std::ofstream fMatchOut(sMatchPath);
                    for (int i = 0; i < coeffSel_tmpt->points.size(); i++)
                    {
                        if (point_selected_surf[i] && (res_last[i] <= 2.0))
                        {
                            fMatchOut << i << " " << feats_down_updated->points[i].x << " " << feats_down_updated->points[i].y << " " << feats_down_updated->points[i].z << " " << 255 << " " << " " << 0  << " " << 0 <<   std::endl;
                            for(auto tmp: Nearest_Points[i])
                                fMatchOut << i << " " << tmp.x << " " << tmp.y << " " << tmp.z <<  " " << 255 << " " << 255 <<  " " << 0 << std::endl;
                            laserCloudOri->push_back(feats_down->points[i]);
                            coeffSel->push_back(coeffSel_tmpt->points[i]);
                            total_residual += res_last[i];
                            laserCloudSelNum++;
                        }
                    }
                    res_mean_last = total_residual / laserCloudSelNum;
                    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                    Eigen::MatrixXd Hsub(laserCloudSelNum, 6);
                    Eigen::VectorXd meas_vec(laserCloudSelNum);
                    Hsub.setZero();

                    for (int i = 0; i < laserCloudSelNum; i++)
                    {
                        const PointType& laser_p = laserCloudOri->points[i];
                        Eigen::Vector3d  point_this(laser_p.x, laser_p.y, laser_p.z);
                        //point_this += Lidar_offset_to_IMU;
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

                        /*** only run in initialization period ***/
                        set_initial_state_cov(g_lio_state);
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
                        //print_dash_board();
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
                    g_lio_state.last_update_time = Measures.lidar.endTime;
                    euler_cur = RotMtoEuler(g_lio_state.rot_end);

                    std::cout << "Frame id: " << Measures.lidar.frameId << " " << iterCount << std::endl;
                    std::cout << "Pos: " << "\t" << g_lio_state.pos_end << std::endl;
                    std::cout << "rot: " << "\t" << g_lio_state.rot_end << std::endl;
                    /*** Rematch Judgement ***/
                    rematch_en = false;
                    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iMaxIteration - 2))))
                    {
                        rematch_en = true;
                        rematch_num++;
                    }
                    BaseCloudPtr world_lidar(new BaseCloud());
                    transformLidar(feats_down, world_lidar);
                    PlyIo fullOriginPly3(m_sOptimRootPath + "after_" + to_string(Measures.lidar.frameId) + "_" + to_string(iterCount) + ".ply");
                    fullOriginPly3.SetOnePatch(*world_lidar);
                    fullOriginPly3.Flush();
                    fullOriginPly3.Close();
                    /*** Convergence Judgements and Covariance Update ***/
                    // if (rematch_num >= 10 || (iterCount == NUM_MAX_ITERATIONS - 1))
                    if (rematch_num >= 2 || (iterCount == m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iMaxIteration - 1)) // Fast lio ori version.
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

                    /*cout << "Match cost time: " << match_time * 1000.0
                         << ", search cost time: " << kdtree_search_time*1000.0
                         << ", PCA cost time: " << pca_time*1000.0
             << ", point num: " << coeffSel_tmpt->size()
                         << ", solver_cost: " << solve_time * 1000.0 << endl;
                    cout <<"Iter cost time: " << tim.toc("Iter") << endl;*/
                }
                /*** add new frame points to voxel map  ***/
                PointVector points_history;
                ikdtree.acquire_removed_points(points_history);

                memset(cube_updated, 0, sizeof(cube_updated));

                for (int i = 0; i < points_history.size(); i++)
                {
                    PointType& pointSel = points_history[i];

                    int cubeI = int((pointSel.x + 0.5 * cube_len) / cube_len) + laserCloudCenWidth;
                    int cubeJ = int((pointSel.y + 0.5 * cube_len) / cube_len) + laserCloudCenHeight;
                    int cubeK = int((pointSel.z + 0.5 * cube_len) / cube_len) + laserCloudCenDepth;

                    if (pointSel.x + 0.5 * cube_len < 0)
                        cubeI--;
                    if (pointSel.y + 0.5 * cube_len < 0)
                        cubeJ--;
                    if (pointSel.z + 0.5 * cube_len < 0)
                        cubeK--;

                    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth)
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        featsArray[cubeInd]->push_back(pointSel);
                    }
                }

                //for (int i = 0; i < feats_down_size; i++)
                //{
                //    /* transform to world frame */
                //    pointBodyToWorld(&(feats_down->points[i]), &(feats_down_updated->points[i]));
                //}
               
                /*ikdtree.Add_Points(feats_down_updated->points, true);*/
                transformLidar(feats_down, m_bUpdateBaseCloud);
                PlyIo fullOriginPly3(m_sLogRootPath + "after_" + to_string(Measures.lidar.frameId) +  ".ply");
                fullOriginPly3.SetOnePatch(*m_bUpdateBaseCloud);
                fullOriginPly3.Flush();
                fullOriginPly3.Close();

                updateVoxelMap(m_bUpdateBaseCloud, m_voxelOptionsCfgPtr->s_dVoxelSize, m_uVoxelMap);
                //SaveVoxelMap(Measures.lidar.frameId);
            }
        }

    }
    return 0;
}

bool R3LIVE::DetectRotation(MeasureGroup& meas)
{
    int iNum = 0;

    V3D vMeanAcc, vCurAcc, cov_acc;
    vMeanAcc << double(meas.imu.front().acc[0]), double(meas.imu.front().acc[1]), double(meas.imu.front().acc[2]);
    cov_acc << 0, 0, 0;
    for (auto& aImu : meas.imu)
    {
        iNum++;
        vCurAcc << float(aImu.acc[0]), aImu.acc[1], aImu.acc[2];
        vMeanAcc += (vCurAcc - vMeanAcc) / iNum;
        cov_acc = cov_acc * (iNum - 1.0) / iNum +
            (vCurAcc - vMeanAcc).cwiseProduct(vCurAcc - vMeanAcc) *
            (iNum - 1.0) / (iNum * iNum);

    }
    if (cov_acc.x() > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dStaticParam && cov_acc.y() > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dStaticParam && cov_acc.z() > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dStaticParam)
    {
        meas.isStatic = false;
    }
    else
        meas.isStatic = true;
    return true;
}

bool R3LIVE::DetectRotation(Eigen::Matrix3d rotation_matrix, double dt)
{
    double xyzrpy[6];
    pcl::getTranslationAndEulerAngles(Eigen::Affine3d(rotation_matrix), xyzrpy[0], xyzrpy[1], xyzrpy[2], xyzrpy[3], xyzrpy[4], xyzrpy[5]);
    double ang = std::sqrt(xyzrpy[3] * xyzrpy[3] + xyzrpy[4] * xyzrpy[4] + xyzrpy[5] * xyzrpy[5]) * 180. / M_PI;
    if (ang > 5)
    {
        LOG(INFO) << "Predic--------------------Rotation: " << endl;
        return true;
    }
    return false;
}

bool R3LIVE::start()
{
    stop();
    m_pThread = new boost::thread(boost::bind(&R3LIVE::service_LIO_update, this));
    return true;
}



bool R3LIVE::stop()
{
    if (m_pThread) {
        m_pThread->join();
        delete m_pThread;
        m_pThread = NULL;
    }
    return true;
}

bool R3LIVE::push_back(sc::LidarFrame& lidarframe)
{
    m_mLidarmutex.lock();
    m_dLidarBuffer.push_back(lidarframe);
    m_mLidarmutex.unlock();
    return true;
}

bool R3LIVE::push_back(sc::ImuFrame& imuframe)
{
    m_dLastImuTime = imuframe.time;
    m_mImumutex.lock();
    m_dImuBuffer.push_back(imuframe);
    m_mImumutex.unlock();
    return true;
}

bool R3LIVE::JugeStatic(MeasureGroup& meas)
{
    int iNum = 0;

    V3D vMeanAcc, vCurAcc, cov_acc;
    vMeanAcc << double(meas.imu.front().acc[0]), double(meas.imu.front().acc[1]), double(meas.imu.front().acc[2]);
    cov_acc << 0, 0, 0;
    for (auto& aImu : meas.imu)
    {
        iNum++;
        vCurAcc << float(aImu.acc[0]), aImu.acc[1], aImu.acc[2];
        vMeanAcc += (vCurAcc - vMeanAcc) / iNum;
        cov_acc = cov_acc * (iNum - 1.0) / iNum +
            (vCurAcc - vMeanAcc).cwiseProduct(vCurAcc - vMeanAcc) *
            (iNum - 1.0) / (iNum * iNum);

    }
    LOG(INFO) << meas.lidar.frameId << " " << cov_acc.x() << " " << cov_acc.y() << " " << cov_acc.z() << std::endl;
    if (cov_acc.x() > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dStaticParam || cov_acc.y() > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dStaticParam || cov_acc.z() > m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_dStaticParam)
    {
        meas.isStatic = false;
        return false;
    }
    else
        meas.isStatic = true;
    return true;
}

R3LIVE::R3LIVE()
{
    m_pThread = nullptr;
    
    featsFromMap = boost::make_shared<BaseCloud>();
    cube_points_add = boost::make_shared<BaseCloud>();
    laserCloudFullRes2 = boost::make_shared<BaseCloud>();
    laserCloudFullResColor = boost::make_shared<BaseCloud>();
    m_bUpdateBaseCloud = boost::make_shared<BaseCloud>();
    m_bVoxelBaseCloud = boost::make_shared<BaseCloud>();
    XAxisPoint_body = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);
    XAxisPoint_world = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);

    downSizeFilterSurf.setLeafSize(0.3, 0.3, 0.4);
    downSizeFilterSurfMin.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    m_baseOptionCfgPtr = std::make_shared<slamBaseOptionsCfg>();
    m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr = std::make_shared<slamCommonOptionsCfg>();
    m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr = std::make_shared<slamR3liveOptionCfg>();
    m_voxelOptionsCfgPtr = std::make_shared<slamVoxelOptionsCfg>();
    cube_len = 50000000.;
    m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_eRL2e << 0.9999, -0.0018, -0.0021, 0.0028, 0.7621, 0.64735, 0.0004, -0.6473, 0.76218;
    m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_eTL2e << -0.0200, 0.0870, 0.1280;
    Eigen::Vector3d veTmp = Eigen::Vector3d::Zero();
    switch(m_baseOptionCfgPtr->s_sSlamR3liveOptionCfgPtr->s_iFindType)
    {
    case 0:
        m_vNearbyGrid.push_back(Eigen::Vector3d::Zero());
        break;
    case 1:
        m_vNearbyGrid = { Eigen::Vector3d(0, 0, 0),  Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
                         Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1) };
        break;
    case 2:
        m_vNearbyGrid = {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 1, 0),
        Eigen::Vector3d(-1, 1, 0), Eigen::Vector3d(1, -1, 0), Eigen::Vector3d(-1, -1, 0), Eigen::Vector3d(1, 0, 1),
        Eigen::Vector3d(-1, 0, 1), Eigen::Vector3d(1, 0, -1), Eigen::Vector3d(-1, 0, -1), Eigen::Vector3d(0, 1, 1),
        Eigen::Vector3d(0, -1, 1), Eigen::Vector3d(0, 1, -1), Eigen::Vector3d(0, -1, -1)};
    case 3:
        m_vNearbyGrid = { Eigen::Vector3d(0, 0, 0),   Eigen::Vector3d(-1, 0, 0),  Eigen::Vector3d(1, 0, 0),   Eigen::Vector3d(0, 1, 0),
                         Eigen::Vector3d(0, -1, 0),  Eigen::Vector3d(0, 0, -1),  Eigen::Vector3d(0, 0, 1),   Eigen::Vector3d(1, 1, 0),
                         Eigen::Vector3d(-1, 1, 0),  Eigen::Vector3d(1, -1, 0),  Eigen::Vector3d(-1, -1, 0), Eigen::Vector3d(1, 0, 1),
                         Eigen::Vector3d(-1, 0, 1),  Eigen::Vector3d(1, 0, -1),  Eigen::Vector3d(-1, 0, -1), Eigen::Vector3d(0, 1, 1),
                         Eigen::Vector3d(0, -1, 1),  Eigen::Vector3d(0, 1, -1),  Eigen::Vector3d(0, -1, -1), Eigen::Vector3d(1, 1, 1),
                         Eigen::Vector3d(-1, 1, 1),  Eigen::Vector3d(1, -1, 1),  Eigen::Vector3d(1, 1, -1),  Eigen::Vector3d(-1, -1, 1),
                         Eigen::Vector3d(-1, 1, -1), Eigen::Vector3d(1, -1, -1), Eigen::Vector3d(-1, -1, -1) };
        
    }

    //m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_sRootPath = "E:\\data\\RTK\\RTK\\ProcessData\\";


}

R3LIVE::R3LIVE(slamBaseOptionsCfg::Ptr r3liveOptionCfg_in)
{

   m_baseOptionCfgPtr = r3liveOptionCfg_in;
   featsFromMap = boost::make_shared<BaseCloud>();
   cube_points_add = boost::make_shared<BaseCloud>();
   laserCloudFullRes2 = boost::make_shared<BaseCloud>();
   laserCloudFullResColor = boost::make_shared<BaseCloud>();

   XAxisPoint_body = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);
   XAxisPoint_world = Eigen::Vector3f(LIDAR_SP_LEN, 0.0, 0.0);

   downSizeFilterSurf.setLeafSize(m_voxel_downsample_size_surf, m_voxel_downsample_size_surf, m_voxel_downsample_size_axis_z);
   downSizeFilterSurfMin.setLeafSize(0.2, 0.2, 0.2);
   downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
   m_pThread = nullptr;
}

void R3LIVE::transformLidar( const BaseCloudPtr& input_cloud, BaseCloudPtr& trans_cloud, int type)
{
    trans_cloud->clear();
    for (size_t i = 0; i < input_cloud->size(); i++) {
        BasePoint p_c = input_cloud->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        if(type)
            p = m_imu_process->Lidar_R_to_IMU * p + m_imu_process->Lidar_offset_to_IMU;
        else
        // p = p_imu->Lid_rot_to_IMU * p + p_imu->Lid_offset_to_IMU;
            p = g_lio_state.rot_end * (/*m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_eRL2e **/ p /*+ m_baseOptionCfgPtr->s_sSlamCommonOptionCfgPtr->s_eTL2e*/) + g_lio_state.pos_end;
        BasePoint pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

void R3LIVE::SaveVoxelMap(int id)
{
    std::vector<Plane> pub_plane_list;
    /*ColorCloudPtr colorCloud(new ColorCloud);
    ColorCloudPtr colorCloudPlane(new ColorCloud);*/
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloudPlane(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    ColorPoint colorPoint;
    for (auto iter = m_uVoxelMap.begin(); iter != m_uVoxelMap.end(); iter++) {
        GetVoxelMap(iter->second, colorCloud, colorCloudPlane);
    }
    pcl::copyPointCloud(*colorCloud, *m_bVoxelBaseCloud);
    pcl::io::savePLYFile(m_sLogRootPath + "voxelmap_" + to_string(id) + ".ply", *colorCloud);
}

void R3LIVE::GetVoxelMap(const OctoTree* current_octo, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorPointCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloudPlane)
{
    if (current_octo == nullptr)
    {
        return;
    }
    if (current_octo->m_bInitOcto )
    {
        //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorPoint
        pcl::PointXYZRGBNormal colorPoint;
        if (current_octo->m_vTempPoints.size() != 0)
        {

            int r = (rand() % 255 + 10) % 255;
            int g = (rand() % 255 + 10) % 255;
            int b = (rand() % 255 + 10) % 255;
            for (auto& pt : current_octo->m_vTempPoints)
            {
                colorPoint.x = pt.point.x;
                colorPoint.y = pt.point.y;
                colorPoint.z = pt.point.z;
                colorPoint.r = r;
                colorPoint.g = g;
                colorPoint.b = b;
                colorPointCloud->points.push_back(colorPoint); 
            }
        }
        return;
    }
}

bool R3LIVE::NearestSearchInVoxel(PointType point, const double voxel_size, int k_nearest, BaseCloud& Nearest_Points, vector<float>& Point_Distance)
{
    BaseCloudPtr SrcCloud(new BaseCloud());
    float loc_xyz[3];
    loc_xyz[0] = point.x / voxel_size;
    loc_xyz[1] = point.y / voxel_size;
    loc_xyz[2] = point.z / voxel_size;
    for (int i = 0; i < m_vNearbyGrid.size(); i++)
    {
        VOXEL_LOC position((int64_t)(loc_xyz[0]) + m_vNearbyGrid[i][0], (int64_t)(loc_xyz[1]) + m_vNearbyGrid[i][1],
            (int64_t)(loc_xyz[2]) + m_vNearbyGrid[i][2]);
        auto iter = m_uVoxelMap.find(position);
        if (iter != m_uVoxelMap.end()) {
            for (int i = 0; i < iter->second->m_vTempPoints.size(); i++)
                SrcCloud->points.push_back(iter->second->m_vTempPoints[i].point);

        }
    }
    if (SrcCloud->points.size() > 5)
    {
        vector<int> indexs;
        pcl::KdTreeFLANN<BasePoint>::Ptr kdtree(new pcl::KdTreeFLANN<BasePoint>);
        kdtree->setInputCloud(SrcCloud);
        kdtree->nearestKSearch(point, k_nearest, indexs, Point_Distance);
        for (auto idx : indexs)
            Nearest_Points.push_back(SrcCloud->points[idx]);
        return 1;
    }
    return 0;

}
