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
	bool                s_bDebugFlag;					   //�Ƿ������־
	std::string         s_sRootPath;                       //���������־�ļ���
	Eigen::Matrix3d     s_eRi2e;                           //imu���������ı궨��R
	Eigen::Vector3d     s_eTi2e;                           //imu���������ı궨��T
	Eigen::Matrix3d     s_eRL2e;                           //���������������ı궨��R
	Eigen::Vector3d     s_eTL2e;                           //���������������ı궨��T
	slamCommonOptionsCfg() :s_bDebugFlag(true) {}
	inline void SetProcessLogPath(const std::string& str_in)
	{
		s_sRootPath = str_in;
	}
};

struct slamR3liveOptionCfg {

	typedef             std::shared_ptr<slamR3liveOptionCfg> Ptr;	            
	int                 s_iPointStep;					    //Դ��Ĳ���
	int                 s_iMaxIteration;					//����������
	double              s_dLidarTimeDelay;                  //�״������imu��ʱ��delay
	double              s_dCameraTimeDelay;                 //��������imu��ʱ��delay
	double              s_dVoxelDownsampleSizeSurf;         //�²���SIZE
	double              s_dMaxmiumKdtreeDis;                //Kdtree ����ʱ���ľ���
	double              s_dMaxmiumResDis;                   //ToDO
	double              s_dPlanarCheckDis;                  //�ж������ֵ
	double              s_dRangPtDis;                       //ToDo
	double              s_dStaticParam;                     //�жϸ�֡�Ƿ�Ϊ��ֹ
	int                 s_iFovRange;
	int                 s_iFindType;                        //voxel ����ģʽ 0-0�� 1-6�� 2-18�� 3-26
	int                 s_iNumMatchPoints;                   //���ڼ��������������
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
	
	int    s_iNumPointsInit;              //voxel��Ҫ���ٸ�����г�ʼ��
	slamVoxelOptionsCfg():s_iMaxPointsSize(100), s_iMaxCovPointsSize(100), s_iMaxLayer(2), s_dVoxelSize(1), s_dDownSampleSize(0.5), s_dPlannarThreshold(0.01), s_dStaticParam(0.005), s_iNumPointsInit(0), s_iUpdateVoxelThreshold(40), s_dUpdateVoxelTimeThreshold(3.){}
};


struct slamColorPointOptionsCfg {
public:
	typedef std::shared_ptr<slamColorPointOptionsCfg> Ptr;
	bool                     s_bUseTime;				   //ʱ�丽ɫ���߾��븽ɫ
	std::string              s_sLidarFilePath;               //las�ļ�·��
	std::string              s_sImagePath;				   //ͼƬ�ļ���
	std::string              s_sPosFilePath;			   //Pos�ļ�·��
	double                   s_dRadius;                    //���븽ɫ����ֵ
	Eigen::Matrix3d          s_eRCam2Lid0;                 //������ı궨��
	Eigen::Vector3d          s_eTCam2Lid0;                 
	Eigen::Matrix3d          s_eRCam2Lid;                  //������ı궨��
	Eigen::Vector3d          s_eTCam2Lid;                  
	Eigen::Matrix3d          s_eRCam2Lid2;                 //������ı궨��
	Eigen::Vector3d          s_eTCam2Lid2;
	Eigen::Matrix3d          s_eCamIntrisic0;              //��������ڲ�
	Eigen::Vector4d          s_eCamDisCoffes0;             //������Ļ������
	Eigen::Matrix3d          s_eCamIntrisic;               //��������ڲ�
	Eigen::Vector4d          s_eCamDisCoffes;              //������Ļ������
	Eigen::Matrix3d          s_eCamIntrisic2;              //��������ڲ�
	Eigen::Vector4d          s_eCamDisCoffes2;             //������Ļ������
	int                      s_iCamNum;
	double                   s_dNdtResolution;             //Ndt�ķֱ���
	double                   s_dPlaneThreshold;

	slamColorPointOptionsCfg() :s_dNdtResolution(0.5), s_dPlaneThreshold(0.4){}
};




struct slamBaseOptionsCfg {
public:
	typedef std::shared_ptr<slamBaseOptionsCfg> Ptr;
	slamCommonOptionsCfg::Ptr     s_sSlamCommonOptionCfgPtr;
	slamR3liveOptionCfg::Ptr      s_sSlamR3liveOptionCfgPtr;
	bool LoadConfigFile(const std::string& path);          //���������ļ�·����ȡ��Ӧ�ı���ֵ

private:
	template <typename T>
	bool GetData(const YAML::Node& node, T& value);        //��ȡyaml�ļ���Ӧ�ı���ֵ
};



#endif // !__OPTIONSCFG_H__


