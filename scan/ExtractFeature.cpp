#include "ExtractFeature.h"

namespace scanframe {
	using namespace sc;
	ExtractFeature::ExtractFeature()
	{
		extractFeatureConfig_ = std::make_shared<ExtractFeatureConfig>();
		extractFeatureConfig_->disA = 0.01;
		extractFeatureConfig_->disB = 0.1;
		extractFeatureConfig_->inf_bound = 10;
		extractFeatureConfig_->n_scans = 16;
		
		extractFeatureConfig_->group_size = 8;
		extractFeatureConfig_->disA = 0.01;
		extractFeatureConfig_->disB = 0.1; // B?
		extractFeatureConfig_->p2l_ratio = 225;
		extractFeatureConfig_->limit_maxmid = 6.25;
		extractFeatureConfig_->limit_midmin = 6.25;
		extractFeatureConfig_->limit_maxmin = 3.24;
		extractFeatureConfig_->jump_up_limit = 170.0;
		extractFeatureConfig_->jump_down_limit = 8.0;
		extractFeatureConfig_->cos160 = 160.0;
		extractFeatureConfig_->edgea = 2;
		extractFeatureConfig_->edgeb = 0.1;
		extractFeatureConfig_->smallp_intersect = 172.5;
		extractFeatureConfig_->smallp_ratio = 1.2;
		extractFeatureConfig_->useExtractFeature = true;
		extractFeatureConfig_->blind = 0.5;

	}
	ExtractFeature::ExtractFeature(ExtractFeatureConfig::Ptr config)
	{
		extractFeatureConfig_ = config;
	}
	void ExtractFeature::GiveFeature(BaseCloudPtr & cloudPtrIn)
	{
		if (cloudPtrIn->empty())
		{
			LOG(INFO) << "input empty pointcloud!";
		}
		
		int all_pts_num = cloudPtrIn->points.size();
		for (int i = 0; i < extractFeatureConfig_->n_scans; i++)
		{
			plBuffer[i].clear();
			//plTypes[i].clear();
			plBuffer[i].reserve(all_pts_num);
			//plTypes[i].reserve(all_pts_num);
		}
		if (extractFeatureConfig_->useExtractFeature)
		{
			//1. 获取每条线上的点
			for (int i = 1; i < all_pts_num; i++)
			{
				BasePoint bPoint = cloudPtrIn->points[i];
				if (abs(cloudPtrIn->points[i].x - abs(cloudPtrIn->points[i - 1].x)) > 1e-7 ||
					abs(cloudPtrIn->points[i].y - abs(cloudPtrIn->points[i - 1].y)) > 1e-7 ||
					abs(cloudPtrIn->points[i].z - abs(cloudPtrIn->points[i - 1].z)) > 1e-7)
				{
					plBuffer[bPoint.ring].push_back(bPoint);
				}
			}
			pcl::PointCloud<pcl::PointXYZRGBNormal> debugCloud;
			//2. 对每条线上的点进行处理
			for (int i = 0; i < extractFeatureConfig_->n_scans; i++)
			{
				BaseCloud& bCloud = plBuffer[i];
				int bCloudSize = plBuffer[i].size();
				std::vector<FeaturePointType>& plFeaturePointType = plTypes[i];
				plFeaturePointType.clear();
				plFeaturePointType.resize(bCloudSize);
				if (plBuffer[i].size() < 5)
					continue;
				bCloudSize--;
				for (int j = 0; j < bCloudSize; j++)
				{
					BasePoint& bPoint = bCloud[j];
					plFeaturePointType[j].range = sqrt(bPoint.x * bPoint.x + bPoint.y * bPoint.y +bPoint.z * bPoint.z);
					double vx = bPoint.x - plBuffer[i][j + 1].x;
					double vy = bPoint.y - plBuffer[i][j + 1].y;
					double vz = bPoint.z - plBuffer[i][j + 1].z;
					plFeaturePointType[j].dista = sqrt(vx * vx + vy * vy + vz * vz);
				}
				
				plFeaturePointType[bCloudSize].dista = sqrt(bCloud[bCloudSize].x * bCloud[bCloudSize].x + bCloud[bCloudSize].y * bCloud[bCloudSize].y + bCloud[bCloudSize].z * bCloud[bCloudSize].z);
				give_feature(bCloud, plFeaturePointType);
				pcl::PointXYZRGBNormal pt;
				for (int i = 0; i < bCloud.size(); i++)
				{
					pt.x = bCloud.points[i].x;
					pt.y = bCloud.points[i].y;
					pt.z = bCloud.points[i].z;
					if (plFeaturePointType[i].ftype == 2)
					{
						pt.r = 0;
						pt.g = 255;
						pt.b = 0;
					}
					else if (plFeaturePointType[i].ftype == 5)
					{
						pt.r = 255;
						pt.g = 0;
						pt.b = 0;
					}
					else {
						pt.r = 255;
						pt.g = 255;
						pt.b = 255;
					}
					debugCloud.push_back(pt);
				}
				
				
			}
		}


	}
	void ExtractFeature::give_feature(BaseCloud& pl, std::vector<FeaturePointType>& types)
	{
		int plsize = pl.size();
		int plsize2;
		if (plsize == 0)
		{
			printf("something wrong\n");
			return;
		}
		int head = 0;

		while (types[head].range < extractFeatureConfig_->blind)
		{
			head++;
		}

		// Surf
		plsize2 = (plsize > extractFeatureConfig_->group_size) ? (plsize - extractFeatureConfig_->group_size) : 0;

		Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
		Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

		int i_nex = 0, i2;
		int last_i = 0; int last_i_nex = 0;
		int last_state = 0;
		int plane_type;

		for (int i = head; i < plsize2; i++)
		{
			if (types[i].range < extractFeatureConfig_->blind)
			{
				continue;
			}

			i2 = i;

			plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
			pl.points[i].normal_x = curr_direct[0];
			pl.points[i].normal_y = curr_direct[1];
			pl.points[i].normal_z = curr_direct[2];
		
			if (plane_type == 1)
			{
				for (int j = i; j <= i_nex; j++)
				{
					if (j != i && j != i_nex)
					{
						types[j].ftype = Real_Plane;
					}
					else
					{
						types[j].ftype = Poss_Plane;
					}
				}

				// if(last_state==1 && fabs(last_direct.sum())>0.5)
				if (last_state == 1 && last_direct.norm() > 0.1)
				{
					double mod = last_direct.transpose() * curr_direct;
					if (mod > -0.707 && mod < 0.707)
					{
						types[i].ftype = Edge_Plane;
					}
					else
					{
						types[i].ftype = Real_Plane;
					}
				}

				i = i_nex - 1;
				last_state = 1;
			}
			else // if(plane_type == 2)
			{
				i = i_nex;
				last_state = 0;
			}

			last_i = i2;
			last_i_nex = i_nex;
			last_direct = curr_direct;
		}

		plsize2 = plsize > 3 ? plsize - 3 : 0;
		for (int i = head + 3; i < plsize2; i++)
		{
			if (types[i].range < extractFeatureConfig_->blind || types[i].ftype >= Real_Plane)
			{
				continue;
			}

			if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
			{
				continue;
			}

			Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
			Eigen::Vector3d vecs[2];

			for (int j = 0; j < 2; j++)
			{
				int m = -1;
				if (j == 1)
				{
					m = 1;
				}

				if (types[i + m].range < extractFeatureConfig_->blind)
				{
					if (types[i].range > extractFeatureConfig_->inf_bound)
					{
						types[i].edj[j] = Nr_inf;
					}
					else
					{
						types[i].edj[j] = Nr_blind;
					}
					continue;
				}

				vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
				vecs[j] = vecs[j] - vec_a;

				types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
				if (types[i].angle[j] < extractFeatureConfig_->jump_up_limit)
				{
					types[i].edj[j] = Nr_180;
				}
				else if (types[i].angle[j] > extractFeatureConfig_->jump_down_limit)
				{
					types[i].edj[j] = Nr_zero;
				}
			}

			types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
			if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
			{
				if (types[i].intersect > extractFeatureConfig_->cos160)
				{
					if (edge_jump_judge(pl, types, i, Prev))
					{
						types[i].ftype = Edge_Jump;
					}
				}
			}
			else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
			{
				if (types[i].intersect > extractFeatureConfig_->cos160)
				{
					if (edge_jump_judge(pl, types, i, Next))
					{
						types[i].ftype = Edge_Jump;
					}
				}
			}
			else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
			{
				if (edge_jump_judge(pl, types, i, Prev))
				{
					types[i].ftype = Edge_Jump;
				}
			}
			else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
			{
				if (edge_jump_judge(pl, types, i, Next))
				{
					types[i].ftype = Edge_Jump;
				}

			}
			else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
			{
				if (types[i].ftype == Nor)
				{
					types[i].ftype = Wire;
				}
			}
		}

		plsize2 = plsize - 1;
		double ratio;
		for (int i = head + 1; i < plsize2; i++)
		{
			if (types[i].range < extractFeatureConfig_->blind || types[i - 1].range < extractFeatureConfig_->blind || types[i + 1].range < extractFeatureConfig_->blind)
			{
				continue;
			}

			if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
			{
				continue;
			}

			if (types[i].ftype == Nor)
			{
				if (types[i - 1].dista > types[i].dista)
				{
					ratio = types[i - 1].dista / types[i].dista;
				}
				else
				{
					ratio = types[i].dista / types[i - 1].dista;
				}

				if (types[i].intersect < extractFeatureConfig_->smallp_intersect && ratio < extractFeatureConfig_->smallp_ratio)
				{
					if (types[i - 1].ftype == Nor)
					{
						types[i - 1].ftype = Real_Plane;
					}
					if (types[i + 1].ftype == Nor)
					{
						types[i + 1].ftype = Real_Plane;
					}
					types[i].ftype = Real_Plane;
				}
			}
		}

		int last_surface = -1;
		for (int j = head; j < plsize; j++)
		{
			if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
			{
				if (last_surface == -1)
				{
					last_surface = j;
				}

				if (j == int(last_surface + extractFeatureConfig_->point_filter_num - 1))
				{
					BasePoint ap;
					ap.x = pl[j].x;
					ap.y = pl[j].y;
					ap.z = pl[j].z;
					ap.intensity = pl[j].intensity;
					//ap.curvature = pl[j].curvature;
					plSurf.push_back(ap);

					last_surface = -1;
				}
			}
			else
			{
				if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
				{
					plCorn.push_back(pl[j]);
				}
				if (last_surface != -1)
				{
					BasePoint ap;
					for (int k = last_surface; k < j; k++)
					{
						ap.x += pl[k].x;
						ap.y += pl[k].y;
						ap.z += pl[k].z;
						ap.intensity += pl[k].intensity;
						
					}
					ap.x /= (j - last_surface);
					ap.y /= (j - last_surface);
					ap.z /= (j - last_surface);
					ap.intensity /= (j - last_surface);
					
					plSurf.push_back(ap);
				}
				last_surface = -1;
			}
		}


	}
	int ExtractFeature::plane_judge(const BaseCloud& pl, std::vector<FeaturePointType>& types, int i_cur, int& i_nex, Eigen::Vector3d& curr_direct)
	{
		double group_dis = extractFeatureConfig_->disA * types[i_cur].range + extractFeatureConfig_->disB;
		group_dis = group_dis * group_dis;
		// i_nex = i_cur;

		double two_dis;
		std::vector<double> disarr;
		disarr.reserve(20);

		for (i_nex = i_cur; i_nex < i_cur + extractFeatureConfig_->group_size; i_nex++)
		{
			if (types[i_nex].range < extractFeatureConfig_->blind)
			{
				curr_direct.setZero();
				return 2;
			}
			disarr.push_back(types[i_nex].dista);
		}

		for (;;)
		{
			if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

			if (types[i_nex].range < extractFeatureConfig_->blind)
			{
				curr_direct.setZero();
				return 2;
			}
			vx = pl[i_nex].x - pl[i_cur].x;
			vy = pl[i_nex].y - pl[i_cur].y;
			vz = pl[i_nex].z - pl[i_cur].z;
			two_dis = vx * vx + vy * vy + vz * vz;
			if (two_dis >= group_dis)
			{
				break;
			}
			disarr.push_back(types[i_nex].dista);
			i_nex++;
		}

		double leng_wid = 0;
		double v1[3], v2[3];
		for (int j = i_cur + 1; j < i_nex; j++)
		{
			if ((j >= pl.size()) || (i_cur >= pl.size())) break;
			v1[0] = pl[j].x - pl[i_cur].x;
			v1[1] = pl[j].y - pl[i_cur].y;
			v1[2] = pl[j].z - pl[i_cur].z;

			v2[0] = v1[1] * vz - vy * v1[2];
			v2[1] = v1[2] * vx - v1[0] * vz;
			v2[2] = v1[0] * vy - vx * v1[1];

			double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
			if (lw > leng_wid)
			{
				leng_wid = lw;
			}
		}


		if ((two_dis * two_dis / leng_wid) < extractFeatureConfig_->p2l_ratio)
		{
			curr_direct.setZero();
			return 0;
		}

		int disarrsize = disarr.size();
		for (int j = 0; j < disarrsize - 1; j++)
		{
			for (int k = j + 1; k < disarrsize; k++)
			{
				if (disarr[j] < disarr[k])
				{
					leng_wid = disarr[j];
					disarr[j] = disarr[k];
					disarr[k] = leng_wid;
				}
			}
		}

		if (disarr[disarr.size() - 2] < 1e-16)
		{
			curr_direct.setZero();
			return 0;
		}

		
		
			double dismax_min = disarr[0] / disarr[disarrsize - 2];
			if (dismax_min >= extractFeatureConfig_->limit_maxmin)
			{
				curr_direct.setZero();
				return 0;
			}
		

		curr_direct << vx, vy, vz;
		curr_direct.normalize();
		return 1;
	}
	bool ExtractFeature::edge_jump_judge(const BaseCloud& pl, std::vector<FeaturePointType>& types, int i, Surround nor_dir)
	{
		if (nor_dir == 0)
		{
			if (types[i - 1].range < extractFeatureConfig_->blind || types[i - 2].range < extractFeatureConfig_->blind)
			{
				return false;
			}
		}
		else if (nor_dir == 1)
		{
			if (types[i + 1].range < extractFeatureConfig_->blind || types[i + 2].range < extractFeatureConfig_->blind)
			{
				return false;
			}
		}
		double d1 = types[i + nor_dir - 1].dista;
		double d2 = types[i + 3 * nor_dir - 2].dista;
		double d;

		if (d1 < d2)
		{
			d = d1;
			d1 = d2;
			d2 = d;
		}

		d1 = sqrt(d1);
		d2 = sqrt(d2);


		if (d1 > extractFeatureConfig_->edgea * d2 || (d1 - d2) > extractFeatureConfig_->edgeb)
		{
			return false;
		}

		return true;
	}


}