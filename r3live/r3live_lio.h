#ifndef __R3LIVE_LIO_H__
#define __R3LIVE_LIO_H__

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include "internal/so3_math.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "internal/common_lib.h"
#include <opencv2/core/eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>
#include "IMU_Processing.h"
#include "kdtree/ikd_Tree.h"
#include "internal/OptionsCfg.h"
#include <glog/logging.h>
#include "voxel_map_util.h"
#define THREAD_SLEEP_TIM 1


#define INIT_TIME (0)
// #define LASER_POINT_COV (0.0015) // Ori
#define LASER_POINT_COV (0.00015)    
#define NUM_MATCH_POINTS 5

#define MAXN 360000
const int laserCloudWidth = 48;
const int laserCloudHeight = 48;
const int laserCloudDepth = 48;

const int laserCloudNum = ::laserCloudWidth * ::laserCloudHeight * ::laserCloudDepth;
//estimator inputs and output;

extern double g_lidar_star_tim;






class R3LIVE
{

public:
    R3LIVE();
    ~R3LIVE() {};
    R3LIVE(slamBaseOptionsCfg::Ptr r3liveOptionCfg_in);
    
    bool push_back(sc::LidarFrame& lidarframe);
    bool push_back(sc::ImuFrame& imuframe);
    bool JugeStatic(MeasureGroup& meas);
    bool start();
    void transformLidar( const BaseCloudPtr& input_cloud, BaseCloudPtr& trans_cloud, int type = 0);
    void SaveVoxelMap(int id);
    void GetVoxelMap(const OctoTree* current_octo, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorPointCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloudPlane);

    bool NearestSearchInVoxel(PointType point, const double voxel_size, int k_nearest, BaseCloud& Nearest_Points, vector<float>& Point_Distance);

    template <typename T>
    void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi, Eigen::Matrix<T, 3, 1>& po)
    {
        Eigen::Vector3d p_body(pi[0], pi[1], pi[2]);
        Eigen::Vector3d p_global(g_lio_state.rot_end * (p_body /*+ Lidar_offset_to_IMU*/)+g_lio_state.pos_end);
        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::mutex                      m_mliomutex;
    std::shared_ptr<ImuProcess>     m_imu_process;
    std::deque<sc::LidarFrame>      m_dLidarBuffer;
    std::deque<sc::ImuFrame>        m_dImuBuffer;
    bool                            m_bLidarPushed = false;
    double                          m_dLastImuTime = -1;
    int                             laserCloudCenWidth = 24;
    int                             laserCloudCenHeight = 24;
    int                             laserCloudCenDepth = 24;
    int                             laserCloudValidNum = 0;
    int                             laserCloudSelNum = 0;


    bool                            flg_exit = false;
    bool                            flg_reset = false;
    bool                            use_encode = true;
    bool                            isRotationMode = true;
    bool                            isRote = false;
    boost::thread* m_pThread;
    MeasureGroup                    Measures;
    StatesGroup                     g_lio_state;
    std::mutex                      m_mLidarmutex;
    std::mutex                      m_mImumutex;
    std::string                     m_sLogRootPath;
    std::string                     m_sOptimRootPath;
    std::string                     m_sMatchRootPath;
    vector<Eigen::Vector3d>         m_vNearbyGrid;
    // Buffers for measurements
    double                          cube_len = 10000000.0;
    double                          last_timestamp_lidar = -1;
    slamBaseOptionsCfg::Ptr         m_baseOptionCfgPtr;
    slamVoxelOptionsCfg::Ptr        m_voxelOptionsCfgPtr;
    std::unordered_map<VOXEL_LOC, OctoTree*> m_uVoxelMap;
    BaseCloudPtr                    m_bUpdateBaseCloud;        //用于更新voxelmap
    BaseCloudPtr                    m_bVoxelBaseCloud;         //voxelMap中的所有点
    

    double HALF_FOV_COS = 0.0;
    double FOV_DEG = 0.0;
    double res_mean_last = 0.05;
    double total_distance = 0.0;
    Eigen::Vector3d position_last = Eigen::Vector3d::Zero();
    double copy_time, readd_time, fov_check_time, readd_box_time, delete_box_time;


    int lidarbuffer_size = 0, imubuffer_size = 0, encodebuffer_size = 0;
    //surf feature in map
    BaseCloudPtr featsFromMap;
    BaseCloudPtr cube_points_add;
    BaseCloudPtr laserCloudFullRes2;
    BaseCloudPtr featsArray[::laserCloudNum];
    BaseCloudPtr laserCloudFullResColor;

    Eigen::Vector3f XAxisPoint_body; //(LIDAR_SP_LEN, 0.0, 0.0);
    Eigen::Vector3f XAxisPoint_world; //(LIDAR_SP_LEN, 0.0, 0.0);

    std::vector<BoxPointType> cub_needrm;
    std::vector<BoxPointType> cub_needad;


    bool _last_inFOV[::laserCloudNum];
    bool now_inFOV[::laserCloudNum];
    bool cube_updated[::laserCloudNum];
    int laserCloudValidInd[::laserCloudNum];


    KD_TREE ikdtree;




    bool dense_map_en, flg_EKF_inited = 0, flg_map_initialized = 0, flg_EKF_converged = 0;
    int effect_feat_num = 0, frame_num = 0;
    double filter_size_corner_min, m_voxel_downsample_size_surf, filter_size_map_min, fov_deg, deltaT, deltaR, aver_time_consu = 0, frame_first_pt_time = 0;
    double m_voxel_downsample_size_axis_z;

    BasePoint pointOri, pointSel, coeff;
    pcl::VoxelGrid<BasePoint> downSizeFilterSurf;
    pcl::VoxelGrid<BasePoint> downSizeFilterSurfMin;
    pcl::VoxelGrid<BasePoint> downSizeFilterMap;
    std::vector<double> m_initial_pose;


    int m_if_estimate_i2c_extrinsic = 1;
    int m_if_estimate_intrinsic = 1;
    int m_if_pub_raw_img = 1;
    int esikf_iter_times = 2;
    std::string m_map_output_dir;
    


    

    PlyIo* plyer = NULL;
    PlyIo* plyerInit = NULL;
    bool isDebug = false;
    int m_max_feature_points = 2000;
    int m_min_feature_points = 200;
    int m_merge_num = 2;
    float m_max_range = 0;
    float m_min_range = 0;
    float m_indoor_downsample_dis = 6.f;

    double cov_omg_n = 0.1;
    double cov_acc_n = 0.4;
    double cov_gry_n = 0.2;
    double cov_bias_acc_n = 0.05;
    double cov_bias_gry_n = 0.1;
    



    //project lidar frame to world
    void pointBodyToWorld(PointType const* const pi, PointType* const po);
    void set_initial_state_cov(StatesGroup& state);
    bool sync_packages(MeasureGroup& meas, int num);
    void RGBpointBodyToWorld(PointType const* const pi, BasePoint* const po);
    int get_cube_index(const int& i, const int& j, const int& k);
    bool center_in_FOV(Eigen::Vector3f cube_p);
    bool if_corner_in_FOV(Eigen::Vector3f cube_p);
    void lasermap_fov_segment();
    int service_LIO_update();
    bool DetectRotation(MeasureGroup& meas);
    bool DetectRotation(Eigen::Matrix3d rotation_matrix, double dt);
    bool stop();

};
#endif
