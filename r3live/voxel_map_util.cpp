#include "voxel_map_util.h"

 
bool OctoTree::octo_knn(BasePoint pt, std::vector<BasePoint> nearest_pts)
{
    
    return false;
}

void OctoTree::init_voxel(const std::vector<pointWithCov>& points, Plane* plane)
{
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = points.size();
    plane->radius = 0;
    for (auto pv : points) {
        plane->covariance += pv.position * pv.position.transpose();
        plane->center += pv.position;
    }
    //计算voxel的中心点和方差
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size -
        plane->center * plane->center.transpose();
}


void OctoTree::init_octo_tree() {
        if (m_vTempPoints.size() > m_sVoxelOptionsCfgPtr->s_iNumPointsInit) {
            std::sort(m_vTempPoints.begin(), m_vTempPoints.end(), [](const pointWithCov& l, const pointWithCov& r) {return l.point.time < r.point.time; });
            init_voxel(m_vTempPoints, m_pPlane);
            m_dStartTime = m_vTempPoints.begin()->point.time;
            m_NewPointsNum = 0;
            m_bInitOcto = true;
        }
    }

void OctoTree::UpdateOctoTree(const pointWithCov& pv)
{
    if (m_bInitOcto)
    {
        m_iAllPointsNum++;
        //更新准则1. 当Voxel的数量大于阈值并且时间超过阈值则进行更新，否者对voxel进行加点操作
        if (m_iAllPointsNum > m_sVoxelOptionsCfgPtr->s_iUpdateVoxelThreshold)
        {
            //m_vTempPoints.erase(m_vTempPoints.begin());
            m_vTempPoints.emplace_back(pv);
            //std::sort(m_vTempPoints.begin(), m_vTempPoints.end(), [](const pointWithCov& l, const pointWithCov& r) {return l.point.time < r.point.time; });
            m_iAllPointsNum = m_vTempPoints.size();
        }
        
    }
    else {
        m_vTempPoints.push_back(pv);
        init_octo_tree();
    }
}



void mapJet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g,
    uint8_t& b) {
    r = 255;
    g = 255;
    b = 255;

    if (v < vmin) {
        v = vmin;
    }

    if (v > vmax) {
        v = vmax;
    }

    double dr, dg, db;

    if (v < 0.1242) {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    }
    else if (v < 0.3747) {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    }
    else if (v < 0.6253) {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    }
    else if (v < 0.8758) {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    }
    else {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }

    r = (uint8_t)(255 * dr);
    g = (uint8_t)(255 * dg);
    b = (uint8_t)(255 * db);
}

void buildVoxelMap(const std::vector<pointWithCov>& input_points, const float voxel_size,
     std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map) {
    int plsize = input_points.size();
    for (int i = 0; i < plsize; i++) {
        const pointWithCov p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.position[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
            (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            feat_map[position]->m_vTempPoints.push_back(p_v);
            feat_map[position]->m_iAllPointsNum++;
        }
        else {
            OctoTree* octo_tree =
                new OctoTree();
            feat_map[position] = octo_tree;
            feat_map[position]->m_vTempPoints.push_back(p_v);
            feat_map[position]->m_iAllPointsNum++;
        }
    }
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
        iter->second->init_octo_tree();
    }
}

void updateVoxelMap(BaseCloudPtr CloudIn, const float voxel_size, std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map)
{
    pointWithCov pv_tmp;
    vector<pointWithCov>            m_vUpdatePointCov;         //用于更新voxelmap
    for (int i = 0; i < CloudIn->points.size(); i++)
    {
        pv_tmp.position = Eigen::Vector3d(CloudIn->points[i].x, CloudIn->points[i].y, CloudIn->points[i].z);
        pv_tmp.point = CloudIn->points[i];
        m_vUpdatePointCov.push_back(pv_tmp);
    }
    int plsize = m_vUpdatePointCov.size();
    for (int i = 0; i < plsize; i++) {
        const pointWithCov p_v = m_vUpdatePointCov[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.position[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
            (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            feat_map[position]->UpdateOctoTree(p_v);
        }
        else {
            OctoTree* octo_tree = new OctoTree();
            feat_map[position] = octo_tree;
            
            feat_map[position]->UpdateOctoTree(p_v);
        }
    }

}









