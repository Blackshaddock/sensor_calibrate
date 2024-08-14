#include "voxel_map_util.h"




    // check is plane , calc plane parameters including plane covariance
void OctoTree::init_plane(const std::vector<pointWithCov>& points, Plane* plane) {
        plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
        plane->covariance = Eigen::Matrix3d::Zero();
        plane->center = Eigen::Vector3d::Zero();
        plane->normal = Eigen::Vector3d::Zero();
        plane->points_size = points.size();
        plane->radius = 0;
        for (auto pv : points) {
            plane->covariance += pv.point * pv.point.transpose();
            plane->center += pv.point;
        }

        //https://blog.csdn.net/jacken123456/article/details/131181592 ���Ƽ���Э����
        plane->center = plane->center / plane->points_size;
        plane->covariance = plane->covariance / plane->points_size -
            plane->center * plane->center.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        int evalsMid = 3 - evalsMin - evalsMax;
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
        Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

        // plane covariance calculation
        Eigen::Matrix3d J_Q;
        J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
            1.0 / plane->points_size;
        if (evalsReal(evalsMin) < planer_threshold_) {
            std::vector<int> index(points.size());
            std::vector<Eigen::Matrix<double, 6, 6>> temp_matrix(points.size());
            for (int i = 0; i < points.size(); i++) {
                Eigen::Matrix<double, 6, 3> J;
                Eigen::Matrix3d F;
                for (int m = 0; m < 3; m++) {
                    if (m != (int)evalsMin) {
                        Eigen::Matrix<double, 1, 3> F_m = (points[i].point - plane->center).transpose() / ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                            (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                                evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                        F.row(m) = F_m;
                    }
                    else {
                        Eigen::Matrix<double, 1, 3> F_m;
                        F_m << 0, 0, 0;
                        F.row(m) = F_m;
                    }
                }
                J.block<3, 3>(0, 0) = evecs.real() * F;
                J.block<3, 3>(3, 0) = J_Q;
                plane->plane_cov += J * points[i].cov * J.transpose();
            }

            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) + plane->normal(2) * plane->center(2));
            plane->is_plane = true;
            if (plane->last_update_points_size == 0) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            }
            else if (plane->points_size - plane->last_update_points_size > 100) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            }

            if (!plane->is_init) {
                plane->id = plane_id;
                plane_id++;
                plane->is_init = true;
            }

        }
        else {
            if (!plane->is_init) {
                plane->id = plane_id;
                plane_id++;
                plane->is_init = true;
            }
            if (plane->last_update_points_size == 0) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            }
            else if (plane->points_size - plane->last_update_points_size > 100) {
                plane->last_update_points_size = plane->points_size;
                plane->is_update = true;
            }
            plane->is_plane = false;
            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +plane->normal(2) * plane->center(2));
        }
    }

// only updaye plane normal, center and radius with new points
void OctoTree::update_plane(const std::vector<pointWithCov>& points, Plane* plane) {
        Eigen::Matrix3d old_covariance = plane->covariance;
        Eigen::Vector3d old_center = plane->center;
        Eigen::Matrix3d sum_ppt =
            (plane->covariance + plane->center * plane->center.transpose()) *
            plane->points_size;
        Eigen::Vector3d sum_p = plane->center * plane->points_size;
        for (size_t i = 0; i < points.size(); i++) {
            Eigen::Vector3d pv = points[i].point;
            sum_ppt += pv * pv.transpose();
            sum_p += pv;
        }
        plane->points_size = plane->points_size + points.size();
        plane->center = sum_p / plane->points_size;
        plane->covariance = sum_ppt / plane->points_size -
            plane->center * plane->center.transpose();
        Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3d::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        int evalsMid = 3 - evalsMin - evalsMax;
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
        Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
        if (evalsReal(evalsMin) < planer_threshold_) {
            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
                evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->d = -(plane->normal(0) * plane->center(0) +
                plane->normal(1) * plane->center(1) +
                plane->normal(2) * plane->center(2));

            plane->is_plane = true;
            plane->is_update = true;
        }
        else {
            plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                evecs.real()(2, evalsMin);
            plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
                evecs.real()(2, evalsMid);
            plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
                evecs.real()(2, evalsMax);
            plane->min_eigen_value = evalsReal(evalsMin);
            plane->mid_eigen_value = evalsReal(evalsMid);
            plane->max_eigen_value = evalsReal(evalsMax);
            plane->radius = sqrt(evalsReal(evalsMax));
            plane->d = -(plane->normal(0) * plane->center(0) +
                plane->normal(1) * plane->center(1) +
                plane->normal(2) * plane->center(2));
            plane->is_plane = false;
            plane->is_update = true;
        }
    }

void OctoTree::init_octo_tree() {
        if (temp_points_.size() > max_plane_update_threshold_) {
            init_plane(temp_points_, plane_ptr_);
            if (plane_ptr_->is_plane == true) {
                octo_state_ = 0;
                if (temp_points_.size() > max_cov_points_size_) {
                    update_cov_enable_ = false;
                }
                if (temp_points_.size() > max_points_size_) {
                    update_enable_ = false;
                }
            }
            else {
                octo_state_ = 1;
                cut_octo_tree();
            }
            init_octo_ = true;
            new_points_num_ = 0;
            //      temp_points_.clear();
        }
    }

void OctoTree::cut_octo_tree() {
        if (layer_ >= max_layer_) {
            octo_state_ = 0;
            return;
        }
        for (size_t i = 0; i < temp_points_.size(); i++) {
            int xyz[3] = { 0, 0, 0 };
            if (temp_points_[i].point[0] > voxel_center_[0]) {
                xyz[0] = 1;
            }
            if (temp_points_[i].point[1] > voxel_center_[1]) {
                xyz[1] = 1;
            }
            if (temp_points_[i].point[2] > voxel_center_[2]) {
                xyz[2] = 1;
            }
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
            if (leaves_[leafnum] == nullptr) {
                leaves_[leafnum] = new OctoTree(
                    max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                    max_cov_points_size_, planer_threshold_);
                leaves_[leafnum]->voxel_center_[0] =
                    voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                leaves_[leafnum]->voxel_center_[1] =
                    voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                leaves_[leafnum]->voxel_center_[2] =
                    voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            }
            leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
            leaves_[leafnum]->new_points_num_++;
        }
        for (int i = 0; i < 8; i++) {
            if (leaves_[i] != nullptr) {
                if (leaves_[i]->temp_points_.size() >
                    leaves_[i]->max_plane_update_threshold_) {
                    init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                    if (leaves_[i]->plane_ptr_->is_plane) {
                        leaves_[i]->octo_state_ = 0;
                    }
                    else {
                        leaves_[i]->octo_state_ = 1;
                        leaves_[i]->cut_octo_tree();
                    }
                    leaves_[i]->init_octo_ = true;
                    leaves_[i]->new_points_num_ = 0;
                }
            }
        }
    }

void OctoTree::UpdateOctoTree(const pointWithCov& pv) {
        if (!init_octo_) {
            new_points_num_++;
            all_points_num_++;
            temp_points_.push_back(pv);
            if (temp_points_.size() > max_plane_update_threshold_) {
                init_octo_tree();
            }
        }
        else {
            if (plane_ptr_->is_plane) {
                if (update_enable_) {
                    new_points_num_++;
                    all_points_num_++;
                    temp_points_.push_back(pv);
                    if (update_cov_enable_) {
                        //temp_points_.push_back(pv);
                    }
                    else {
                        new_points_.push_back(pv);
                    }
                    if (new_points_num_ > update_size_threshold_) {
                        if (update_cov_enable_) {
                            init_plane(temp_points_, plane_ptr_);
                        }
                        new_points_num_ = 0;
                    }
                    if (all_points_num_ >= max_cov_points_size_) {
                        update_cov_enable_ = false;
                        //std::vector<pointWithCov>().swap(temp_points_);
                    }
                    if (all_points_num_ >= max_points_size_) {
                        update_enable_ = false;
                        //plane_ptr_->update_enable = false;
                        //std::vector<pointWithCov>().swap(new_points_);
                    }
                }
                else {
                    return;
                }
            }
            else {
                if (layer_ < max_layer_) {
                    if (temp_points_.size() != 0) {
                        std::vector<pointWithCov>().swap(temp_points_);
                    }
                    if (new_points_.size() != 0) {
                        std::vector<pointWithCov>().swap(new_points_);
                    }
                    int xyz[3] = { 0, 0, 0 };
                    if (pv.point[0] > voxel_center_[0]) {
                        xyz[0] = 1;
                    }
                    if (pv.point[1] > voxel_center_[1]) {
                        xyz[1] = 1;
                    }
                    if (pv.point[2] > voxel_center_[2]) {
                        xyz[2] = 1;
                    }
                    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                    if (leaves_[leafnum] != nullptr) {
                        leaves_[leafnum]->UpdateOctoTree(pv);
                    }
                    else {
                        leaves_[leafnum] = new OctoTree(
                            max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                            max_cov_points_size_, planer_threshold_);
                        leaves_[leafnum]->layer_point_size_ = layer_point_size_;
                        leaves_[leafnum]->voxel_center_[0] =
                            voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                        leaves_[leafnum]->voxel_center_[1] =
                            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                        leaves_[leafnum]->voxel_center_[2] =
                            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                        leaves_[leafnum]->UpdateOctoTree(pv);
                    }
                }
                else {
                    if (update_enable_) {
                        new_points_num_++;
                        all_points_num_++;
                        if (update_cov_enable_) {
                            temp_points_.push_back(pv);
                        }
                        else {
                            new_points_.push_back(pv);
                        }
                        if (new_points_num_ > update_size_threshold_) {
                            if (update_cov_enable_) {
                                init_plane(temp_points_, plane_ptr_);
                            }
                            else {
                                update_plane(new_points_, plane_ptr_);
                                new_points_.clear();
                            }
                            new_points_num_ = 0;
                        }
                        if (all_points_num_ >= max_cov_points_size_) {
                            update_cov_enable_ = false;
                            std::vector<pointWithCov>().swap(temp_points_);
                        }
                        if (all_points_num_ >= max_points_size_) {
                            update_enable_ = false;
                            plane_ptr_->update_enable = false;
                            std::vector<pointWithCov>().swap(new_points_);
                        }
                    }
                }
            }
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

void buildVoxelMap(const std::vector<pointWithCov>& input_points,
    const float voxel_size, const int max_layer,
    const std::vector<int>& layer_point_size,
    const int max_points_size, const int max_cov_points_size,
    const float planer_threshold,
    std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map) {
    int plsize = input_points.size();
    for (int i = 0; i < plsize; i++) {
        const pointWithCov p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
            (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
        }
        else {
            OctoTree* octo_tree =
                new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                    max_cov_points_size, planer_threshold);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
            feat_map[position]->layer_point_size_ = layer_point_size;
        }
    }
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
        iter->second->init_octo_tree();
    }
}

void updateVoxelMap(const std::vector<pointWithCov>& input_points,
    const float voxel_size, const int max_layer,
    const std::vector<int>& layer_point_size,
    const int max_points_size, const int max_cov_points_size,
    const float planer_threshold,
    std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map) {
    int plsize = input_points.size();
    for (int i = 0; i < plsize; i++) {
        const pointWithCov p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
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
            OctoTree* octo_tree =
                new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                    max_cov_points_size, planer_threshold);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->UpdateOctoTree(p_v);
        }
    }
}


void pointBodyToWorld(const StatesGroup& state,   BasePoint const * const pi,  BasePoint* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    p_body = p_body + V3D::Zero();/*Lidar_offset_to_IMU*/
    V3D p_global(state.rot_end * (p_body)+state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}




void transformLidar(const StatesGroup& state,
    const shared_ptr<ImuProcess>& p_imu,
    const BaseCloudPtr& input_cloud,
    BaseCloudPtr& trans_cloud, int type) {
    trans_cloud->clear();
    for (size_t i = 0; i < input_cloud->size(); i++) {
        BasePoint p_c = input_cloud->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        if(type)
            p = p_imu->Lid_rot_to_IMU * p + p_imu->Lid_offset_to_IMU;
        p = state.rot_end * p + state.pos_end;
        BasePoint pi = p_c;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        trans_cloud->points.push_back(pi);
    }
}

void build_single_residual(const pointWithCov& pv, const OctoTree* current_octo,
    const int current_layer, const int max_layer,
    const double sigma_num, bool& is_sucess,
    double& prob, ptpl& single_ptpl) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_world;
    if (current_octo->plane_ptr_->is_plane) {
        Plane& plane = *current_octo->plane_ptr_;
        if (plane.points_size < 20)
            return;
        Eigen::Vector3d p_world_to_center = p_w - plane.center;
        double proj_x = p_world_to_center.dot(plane.x_normal);
        double proj_y = p_world_to_center.dot(plane.y_normal);

        float dis_to_plane =
            fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
                plane.normal(2) * p_w(2) + plane.d);


        float dis_to_center =
            (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
            (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
            (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center;
            J_nq.block<1, 3>(0, 3) = -plane.normal;
            double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
            sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
            if (dis_to_plane <  0.1/*sigma_num * sqrt(abs(sigma_l))*/) {
                is_sucess = true;
                single_ptpl.point = pv.point;
                single_ptpl.plane_cov = plane.plane_cov;
                single_ptpl.normal = plane.normal;
                single_ptpl.center = plane.center;
                single_ptpl.d = plane.d;
                single_ptpl.layer = current_layer;

                
               /* double this_prob = 1.0 / (sqrt(sigma_l)) *
                    exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) 
                {
                    prob = this_prob;
                    single_ptpl.point = pv.point;
                    single_ptpl.plane_cov = plane.plane_cov;
                    single_ptpl.normal = plane.normal;
                    single_ptpl.center = plane.center;
                    single_ptpl.d = plane.d;
                    single_ptpl.layer = current_layer;
                }*/
                return;
            }
            else {
                // is_sucess = false;
                return;
            }
        }
        else {
            // is_sucess = false;
            return;
        }
    }
    else {
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {

                    OctoTree* leaf_octo = current_octo->leaves_[leafnum];
                    build_single_residual(pv, leaf_octo, current_layer + 1, max_layer,
                        sigma_num, is_sucess, prob, single_ptpl);
                }
            }
            return;
        }
        else {
            // is_sucess = false;
            return;
        }
    }
}

void GetUpdatePlane(const OctoTree* current_octo, const int pub_max_voxel_layer,
    std::vector<Plane>& plane_list) {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
    if (current_octo->plane_ptr_->is_update) {
        plane_list.push_back(*current_octo->plane_ptr_);
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer,
                        plane_list);
                }
            }
        }
    }
    return;
}


void BuildResidualListOMP(const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
    const double voxel_size, const double sigma_num,
    const int max_layer,
    const std::vector<pointWithCov>& pv_list,
    std::vector<ptpl>& ptpl_list,
    std::vector<Eigen::Vector3d>& non_match) {
    std::mutex mylock;
    ptpl_list.clear();
    std::vector<ptpl> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size());
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
        useful_ptpl[i] = false;
    }
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < index.size(); i++) {
        pointWithCov pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
            (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            OctoTree* current_octo = iter->second;
            ptpl single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                is_sucess, prob, single_ptpl);
            if (!is_sucess) {
                VOXEL_LOC near_position = position;
                if (loc_xyz[0] >
                    (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                }
                else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                    current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] >
                    (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                }
                else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                    current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] >
                    (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                }
                else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                    current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find(near_position);
                if (iter_near != voxel_map.end()) {
                    build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                        is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {

                mylock.lock();
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
                mylock.unlock();
            }
            else {
                mylock.lock();
                useful_ptpl[i] = false;
                mylock.unlock();
            }
        }
    }
    for (size_t i = 0; i < useful_ptpl.size(); i++) {
        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        }
    }
}

void BuildResidualListNormal(
    const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
    const double voxel_size, const double sigma_num, const int max_layer,
    const std::vector<pointWithCov>& pv_list, std::vector<ptpl>& ptpl_list,
    std::vector<Eigen::Vector3d>& non_match) {
    ptpl_list.clear();
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < pv_list.size(); ++i) {
        pointWithCov pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
            (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            OctoTree* current_octo = iter->second;
            ptpl single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                is_sucess, prob, single_ptpl);

            if (!is_sucess) {
                VOXEL_LOC near_position = position;
                if (loc_xyz[0] >
                    (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                }
                else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                    current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] >
                    (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                }
                else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                    current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] >
                    (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                }
                else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                    current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find(near_position);
                if (iter_near != voxel_map.end()) {
                    build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                        is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {
                ptpl_list.push_back(single_ptpl);
            }
            else {
                non_match.push_back(pv.point_world);
            }
        }
    }
}

void calcBodyCov(Eigen::Vector3d& pb, const float range_inc,
    const float degree_inc, Eigen::Matrix3d& cov) {
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);

    float range_var = range_inc * range_inc;

    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
        pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0,
        -direction(0), -direction(1), direction(0), 0;

    Eigen::Vector3d base_vector1(1, 1,
        -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();

    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();

    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
        base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    cov = direction * range_var * direction.transpose() +
        A * direction_var * A.transpose();
};






