#ifndef __COLOR_POINT_H__
#define __COLOR_POINT_H__
#include "internal/OptionsCfg.h"
#include "internal/Utils.h"
#include "glog/logging.h"
#include "internal/PlyIo.hpp"
#include "internal/Structs.h"
#include <chrono>
//#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/visualization/pcl_visualizer.h"
#include "ndt_omp/ndt_omp.h"
#include "ndt_omp/voxel_grid_covariance_omp.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/circular_buffer.hpp>
#define HASH_V 5000
class Color_LOC {
public:
	int u, v;

		Color_LOC(int vx = 0, int vy = 0)
		: u(vx), v(vy) {}

	bool operator==(const Color_LOC& other) const {
		return (u == other.u && v == other.v);
	}
};

// Hash value
namespace std {
	template <> struct hash<Color_LOC> {
		int64_t operator()(const Color_LOC& s) const {
			using std::hash;
			using std::size_t;
			return (((s.u) * HASH_V) + (s.v));
		}
	};
} // namespace std


using namespace sc;
using namespace pclomp;

struct SurfelPoint {
	double timestamp;
	Eigen::Vector3d point;  // raw data
	Eigen::Vector3d point_in_map;
	size_t plane_id;
};

struct SurfelPlane {
	Eigen::Vector4d p4;
	Eigen::Vector3d Pi; // Closest Point Paramization
	Eigen::Vector3d boxMin;
	Eigen::Vector3d boxMax;
	BaseCloud cloud;
	BaseCloud cloud_inlier;
};


typedef pcl::PointXYZRGB colorPointT;
typedef pcl::PointCloud<colorPointT> colorPointCloudT;
typedef colorPointCloudT::Ptr        colorPointCloudTPtr;



class ColoredPoint {
public:

	typedef std::shared_ptr<ColoredPoint> Ptr;
	ColoredPoint();
	ColoredPoint(slamColorPointOptionsCfg::Ptr confIn);
	
	slamColorPointOptionsCfg::Ptr     m_pConf;
	sc::BaseCloudPtr                  m_pPoints;
	std::vector<sc::ImageFrame>       m_vImgFrame, m_vImgFrameC0, m_vImgFrameC2;

	std::vector<sc::PoseD>            m_vPosFrame;
	static std::string                       m_sLogPath;
	
	static std::vector<std::string>           m_vText;
	static std::vector<cv::Point>			  m_vPoint;
	static int                                m_iCount;    //统计所选2D点的个数
	static string                             m_sCurImgId;
	std::unordered_map<Color_LOC, ColorPoint> m_uColor;
	pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr m_pNdtOmp;
	std::vector<SurfelPlane> m_surPlanes;
	colorPointCloudTPtr m_surMap;
	boost::circular_buffer<int> m_colorList;
	
	
	void LoadPoints();
	void LoadImages();
	void LoadTraj();
	void LoadConf(const std::string str_in);
	void Run();
	void FindNearestPos(double time, int& id);
	void PointsColorful();
	void CloudProjection(BaseCloudPtr cloudIn, ImageFrame imgFrame, ColorCloudPtr cloudout);
	bool MarkImagePoint();
	static void MouseEvent(int event, int x, int y, int flags, void* p);
	void VtkDisplay();
	void NdtInit(double ndtResolution);
	int checkPlaneType(const Eigen::Vector3d& eigen_value, const Eigen::Matrix3d& eigen_vector, const double& p_lambda);
	bool fitPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Vector4d& coeffs, BaseCloudPtr cloud_inliers);
	void initColorList();
	inline void sort_vec(const Eigen::Vector3d& vec,
		Eigen::Vector3d& sorted_vec,
		Eigen::Vector3i& ind) {
		ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size() - 1);//[0 1 2]
		auto rule = [vec](int i, int j)->bool {
			return vec(i) > vec(j);
		};  // regular expression, as a predicate of sort

		std::sort(ind.data(), ind.data() + ind.size(), rule);

		// The data member function returns a pointer to the first element of VectorXd,
		// similar to begin()
		for (int i = 0; i < vec.size(); i++) {
			sorted_vec(i) = vec(ind(i));
		}
	}

};
#endif