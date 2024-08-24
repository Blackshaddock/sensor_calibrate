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
class ColoredPoint {
public:

	typedef std::shared_ptr<ColoredPoint> Ptr;
	ColoredPoint();
	
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
	

};
#endif