#ifndef __OPTIONSCFG_H__
#define __OPTIONSCFG_H__

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
using namespace std;
struct slamCommonOptionsCfg {
public:
	typedef             std::shared_ptr<slamCommonOptionsCfg> Ptr;
	bool                s_bDebugFlag;					   //是否输出日志
	std::string         s_sRootPath;                       //用于输出日志文件等
	Eigen::Matrix3d     s_eRi2e;                           //imu到编码器的标定参R
	Eigen::Vector3d     s_eTi2e;                           //imu到编码器的标定参T
	Eigen::Matrix3d     s_eRL2e;                           //激光器到编码器的标定参R
	Eigen::Vector3d     s_eTL2e;                           //激光器到编码器的标定参T
	slamCommonOptionsCfg() :s_bDebugFlag(true) {}
	inline void SetProcessLogPath(const std::string& str_in)
	{
		s_sRootPath = str_in;
	}
};

struct slamR3liveOptionCfg {

	typedef             std::shared_ptr<slamR3liveOptionCfg> Ptr;	            
	int                 s_iPointStep;					    //源点的步长
	int                 s_iMaxIteration;					//最大迭代次数
	double              s_dLidarTimeDelay;                  //雷达相对于imu的时间delay
	double              s_dCameraTimeDelay;                 //相机相对于imu的时间delay
	double              s_dVoxelDownsampleSizeSurf;         //下采样SIZE
	double              s_dMaxmiumKdtreeDis;                //Kdtree 搜索时最大的距离
	double              s_dMaxmiumResDis;                   //ToDO
	double              s_dPlanarCheckDis;                  //判断面的阈值
	double              s_dRangPtDis;                       //ToDo
	double              s_dStaticParam;                     //判断该帧是否为静止
	int                 s_iFovRange;
	int                 s_iFindType;                        //voxel 查找模式 0-0， 1-6， 2-18， 3-26
	int                 s_iNumMatchPoints;                   //用于计算面的最近点个数
	slamR3liveOptionCfg() :s_iPointStep(1), s_iMaxIteration(4), s_dLidarTimeDelay(0.), s_dCameraTimeDelay(0.), s_dVoxelDownsampleSizeSurf(0.3),
		s_dMaxmiumKdtreeDis(0.5), s_dMaxmiumResDis(0.3), s_dPlanarCheckDis(0.10), s_dRangPtDis(500.), s_dStaticParam(0.005), s_iFovRange(4), s_iFindType(2), s_iNumMatchPoints(5){}



};


struct slamVoxelOptionsCfg {
public:
	typedef std::shared_ptr<slamVoxelOptionsCfg> Ptr;
	vector<double> s_vdLayerPointSize;
	std::vector<int> s_vLayerSize;
	int s_iMaxPointsSize;
	int s_iMaxCovPointsSize;
	int s_iMaxLayer;
	double s_dVoxelSize;
	double s_dDownSampleSize;
	double s_dPlannarThreshold;
	int s_iUpdateVoxelThreshold;
	double s_dUpdateVoxelTimeThreshold;
	double s_dStaticParam;
	
	int    s_iNumPointsInit;              //voxel需要多少个点进行初始化
	slamVoxelOptionsCfg():s_iMaxPointsSize(100), s_iMaxCovPointsSize(100), s_iMaxLayer(2), s_dVoxelSize(1), s_dDownSampleSize(0.5), s_dPlannarThreshold(0.01), s_dStaticParam(0.005), s_iNumPointsInit(0), s_iUpdateVoxelThreshold(40), s_dUpdateVoxelTimeThreshold(3.){}
};


struct slamColorPointOptionsCfg {
public:
	typedef std::shared_ptr<slamColorPointOptionsCfg> Ptr;
	bool                     s_bUseTime;				   //时间附色或者距离附色
	std::string              s_sLidarFilePath;               //las文件路径
	std::string              s_sImagePath;				   //图片文件夹
	std::string              s_sPosFilePath;			   //Pos文件路径
	double                   s_dRadius;                    //距离附色的阈值
	Eigen::Matrix3d          s_eRCam2Lid0;                 //左相机的标定参
	Eigen::Vector3d          s_eTCam2Lid0;                 
	Eigen::Matrix3d          s_eRCam2Lid;                  //主相机的标定参
	Eigen::Vector3d          s_eTCam2Lid;                  
	Eigen::Matrix3d          s_eRCam2Lid2;                 //右相机的标定参
	Eigen::Vector3d          s_eTCam2Lid2;
	Eigen::Matrix3d          s_eCamIntrisic0;              //左相机的内参
	Eigen::Vector4d          s_eCamDisCoffes0;             //左相机的畸变参数
	Eigen::Matrix3d          s_eCamIntrisic;               //主相机的内参
	Eigen::Vector4d          s_eCamDisCoffes;              //主相机的畸变参数
	Eigen::Matrix3d          s_eCamIntrisic2;              //右相机的内参
	Eigen::Vector4d          s_eCamDisCoffes2;             //右相机的畸变参数
	int                      s_iCamNum;
	double                   s_dNdtResolution;             //Ndt的分辨率
	double                   s_dPlaneThreshold;

	slamColorPointOptionsCfg() :s_dNdtResolution(0.5), s_dPlaneThreshold(0.4){}
};




struct slamBaseOptionsCfg {
public:
	typedef std::shared_ptr<slamBaseOptionsCfg> Ptr;
	slamCommonOptionsCfg::Ptr     s_sSlamCommonOptionCfgPtr;
	slamR3liveOptionCfg::Ptr      s_sSlamR3liveOptionCfgPtr;
	bool LoadConfigFile(const std::string& path);          //根据配置文件路径获取对应的变量值

private:
	template <typename T>
	bool GetData(const YAML::Node& node, T& value);        //获取yaml文件对应的变量值
};



#endif // !__OPTIONSCFG_H__


