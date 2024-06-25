#ifndef __EXTRACTFEATURE_H__
#define __EXTRACTFEATURE_H__
#include "../internal/Structs.h"
#include "../internal/PoseD.h"
#include "../internal/Utils.h"
#include "../internal/PlyIo.hpp"
namespace scanframe {
	using namespace sc;
	enum Feature { Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint };
	enum Surround { Prev, Next };
	enum E_jump { Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind };

	struct FeaturePointType
	{
		double range;       //局部坐标系的距离
		double dista;       //与后面一个点的距离
		double angle[2];
		double intersect;
		E_jump edj[2];
		Feature ftype;
		FeaturePointType()
		{
			range = 0;
			edj[Prev] = Nr_nor;
			edj[Next] = Nr_nor;
			ftype = Nor;
			intersect = 2;
		}
	};

	struct ExtractFeatureConfig {
		typedef std::shared_ptr<ExtractFeatureConfig> Ptr;
		int group_size;                                             //多少个点为一组
		int n_scans, point_filter_num;
		double disA, disB, inf_bound;								//距离阈值，判断是否为平面
		double limit_maxmid, limit_midmin, limit_maxmin;
		double p2l_ratio;
		double jump_up_limit, jump_down_limit;
		double cos160;
		double edgea, edgeb;
		double smallp_intersect, smallp_ratio, blind;
		
		bool useExtractFeature;
		
	};
	class ExtractFeature
	{
	public:
		ExtractFeature();
		ExtractFeature(ExtractFeatureConfig::Ptr config);

		void GiveFeature(BaseCloudPtr & cloudPtrIn);

		void give_feature(BaseCloud& pl, std::vector<FeaturePointType>& types);

		int  plane_judge(const BaseCloud& pl, std::vector<FeaturePointType>& types, int i, int& i_nex, Eigen::Vector3d& curr_direct);

		bool edge_jump_judge(const BaseCloud& pl, std::vector<FeaturePointType>& types, int i, Surround nor_dir);

		~ExtractFeature() {};

		BaseCloud plFull, plCorn, plSurf;
		BaseCloud plBuffer[128];
		std::vector<FeaturePointType> plTypes[128];
		double vx, vy, vz;

	private:
		ExtractFeatureConfig::Ptr extractFeatureConfig_;


	};

}
#endif // !__EXTRACTFEATURE_H__


