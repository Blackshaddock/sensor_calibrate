#ifndef VOXEL_MAP_UTIL_H__
#define VOXEL_MAP_UTIL_H__

#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <mutex>
#include "IMU_Processing.h"

#define HASH_P 116101
#define MAX_N 10000000000

static int plane_id = 0;

// a point to plane matching structure
typedef struct ptpl {
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    Eigen::Vector3d center;
    Eigen::Matrix<double, 6, 6> plane_cov;
    double d;
    int layer;
} ptpl;

// 3D point with covariance
typedef struct pointWithCov {
    Eigen::Vector3d point;
    Eigen::Vector3d point_world;
    Eigen::Matrix3d cov;
} pointWithCov;

typedef struct Plane {
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    Eigen::Vector3d y_normal;
    Eigen::Vector3d x_normal;
    Eigen::Matrix3d covariance;
    Eigen::Matrix<double, 6, 6> plane_cov;
    float radius = 0;
    float min_eigen_value = 1;
    float mid_eigen_value = 1;
    float max_eigen_value = 1;
    float d = 0;
    int points_size = 0;

    bool is_plane = false;
    bool is_init = false;
    int id;
    // is_update and last_update_points_size are only for publish plane
    bool is_update = false;
    int last_update_points_size = 0;
    bool update_enable = true;
} Plane;

class VOXEL_LOC {
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC& other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// Hash value
namespace std {
    template <> struct hash<VOXEL_LOC> {
        int64_t operator()(const VOXEL_LOC& s) const {
            using std::hash;
            using std::size_t;
            return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
        }
    };
} // namespace std

class OctoTree {
public:
    std::vector<pointWithCov> temp_points_; // all points in an octo tree
    std::vector<pointWithCov> new_points_;  // new points in an octo tree
    Plane* plane_ptr_;
    int max_layer_;
    bool indoor_mode_;
    int layer_;
    int octo_state_; // 0 is end of tree, 1 is not
    OctoTree* leaves_[8];
    double voxel_center_[3]; // x, y, z
    std::vector<int> layer_point_size_;
    float quater_length_;
    float planer_threshold_;
    int max_plane_update_threshold_;
    int update_size_threshold_;
    int all_points_num_;
    int new_points_num_;
    int max_points_size_;
    int max_cov_points_size_;
    bool init_octo_;
    bool update_cov_enable_;
    bool update_enable_;

    OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
        int max_point_size, int max_cov_points_size, float planer_threshold)
        : max_layer_(max_layer), layer_(layer),
        layer_point_size_(layer_point_size), max_points_size_(max_point_size),
        max_cov_points_size_(max_cov_points_size),
        planer_threshold_(planer_threshold) {
        temp_points_.clear();
        octo_state_ = 0;
        new_points_num_ = 0;
        all_points_num_ = 0;
        // when new points num > 5, do a update
        update_size_threshold_ = 5;
        init_octo_ = false;
        update_enable_ = true;
        update_cov_enable_ = true;
        max_plane_update_threshold_ = layer_point_size_[layer_];
        for (int i = 0; i < 8; i++) {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new Plane;
    }

    // check is plane , calc plane parameters including plane covariance
    void init_plane(const std::vector<pointWithCov>& points, Plane* plane);

    // only updaye plane normal, center and radius with new points
    void update_plane(const std::vector<pointWithCov>& points, Plane* plane);

    void init_octo_tree();

    void cut_octo_tree();

    void UpdateOctoTree(const pointWithCov& pv);
};

    void mapJet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g,
        uint8_t& b);

    void buildVoxelMap(const std::vector<pointWithCov>& input_points,
        const float voxel_size, const int max_layer,
        const std::vector<int>& layer_point_size,
        const int max_points_size, const int max_cov_points_size,
        const float planer_threshold,
        std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map);

    void updateVoxelMap(const std::vector<pointWithCov>& input_points,
        const float voxel_size, const int max_layer,
        const std::vector<int>& layer_point_size,
        const int max_points_size, const int max_cov_points_size,
        const float planer_threshold,
        std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map);

    void pointBodyToWorld(const StatesGroup& state, BasePoint const* const pi, BasePoint* const po);

    template <typename T>
    void pointBodyToWorld(const StatesGroup& state, const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po) {
        V3D p_body(pi[0], pi[1], pi[2]);
        p_body = p_body + Lidar_offset_to_IMU;
        V3D p_global(state.rot_end * (p_body)+state.pos_end);
        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }


    void transformLidar(const StatesGroup& state,
        const shared_ptr<ImuProcess>& p_imu,
        const BaseCloudPtr& input_cloud,
        BaseCloudPtr& trans_cloud);

    void build_single_residual(const pointWithCov& pv, const OctoTree* current_octo,
        const int current_layer, const int max_layer,
        const double sigma_num, bool& is_sucess,
        double& prob, ptpl& single_ptpl);

    void GetUpdatePlane(const OctoTree* current_octo, const int pub_max_voxel_layer,
        std::vector<Plane>& plane_list);

    // void BuildResidualListTBB(const unordered_map<VOXEL_LOC, OctoTree *>
    // &voxel_map,
    //                           const double voxel_size, const double sigma_num,
    //                           const int max_layer,
    //                           const std::vector<pointWithCov> &pv_list,
    //                           std::vector<ptpl> &ptpl_list,
    //                           std::vector<Eigen::Vector3d> &non_match) {
    //   std::mutex mylock;
    //   ptpl_list.clear();
    //   std::vector<ptpl> all_ptpl_list(pv_list.size());
    //   std::vector<bool> useful_ptpl(pv_list.size());
    //   std::vector<size_t> index(pv_list.size());
    //   for (size_t i = 0; i < index.size(); ++i) {
    //     index[i] = i;
    //     useful_ptpl[i] = false;
    //   }
    //   std::for_each(
    //       std::execution::par_unseq, index.begin(), index.end(),
    //       [&](const size_t &i) {
    //         pointWithCov pv = pv_list[i];
    //         float loc_xyz[3];
    //         for (int j = 0; j < 3; j++) {
    //           loc_xyz[j] = pv.point_world[j] / voxel_size;
    //           if (loc_xyz[j] < 0) {
    //             loc_xyz[j] -= 1.0;
    //           }
    //         }
    //         VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
    //                            (int64_t)loc_xyz[2]);
    //         auto iter = voxel_map.find(position);
    //         if (iter != voxel_map.end()) {
    //           OctoTree *current_octo = iter->second;
    //           ptpl single_ptpl;
    //           bool is_sucess = false;
    //           double prob = 0;
    //           build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
    //                                 is_sucess, prob, single_ptpl);
    //           if (!is_sucess) {
    //             VOXEL_LOC near_position = position;
    //             if (loc_xyz[0] > (current_octo->voxel_center_[0] +
    //                               current_octo->quater_length_)) {
    //               near_position.x = near_position.x + 1;
    //             } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
    //                                      current_octo->quater_length_)) {
    //               near_position.x = near_position.x - 1;
    //             }
    //             if (loc_xyz[1] > (current_octo->voxel_center_[1] +
    //                               current_octo->quater_length_)) {
    //               near_position.y = near_position.y + 1;
    //             } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
    //                                      current_octo->quater_length_)) {
    //               near_position.y = near_position.y - 1;
    //             }
    //             if (loc_xyz[2] > (current_octo->voxel_center_[2] +
    //                               current_octo->quater_length_)) {
    //               near_position.z = near_position.z + 1;
    //             } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
    //                                      current_octo->quater_length_)) {
    //               near_position.z = near_position.z - 1;
    //             }
    //             auto iter_near = voxel_map.find(near_position);
    //             if (iter_near != voxel_map.end()) {
    //               build_single_residual(pv, iter_near->second, 0, max_layer,
    //                                     sigma_num, is_sucess, prob, single_ptpl);
    //             }
    //           }
    //           if (is_sucess) {

    //             mylock.lock();
    //             useful_ptpl[i] = true;
    //             all_ptpl_list[i] = single_ptpl;
    //             mylock.unlock();
    //           } else {
    //             mylock.lock();
    //             useful_ptpl[i] = false;
    //             mylock.unlock();
    //           }
    //         }
    //       });
    //   for (size_t i = 0; i < useful_ptpl.size(); i++) {
    //     if (useful_ptpl[i]) {
    //       ptpl_list.push_back(all_ptpl_list[i]);
    //     }
    //   }
    // }

    void BuildResidualListOMP(const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        const double voxel_size, const double sigma_num,
        const int max_layer,
        const std::vector<pointWithCov>& pv_list,
        std::vector<ptpl>& ptpl_list,
        std::vector<Eigen::Vector3d>& non_match);

    void BuildResidualListNormal(
        const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        const double voxel_size, const double sigma_num, const int max_layer,
        const std::vector<pointWithCov>& pv_list, std::vector<ptpl>& ptpl_list,
        std::vector<Eigen::Vector3d>& non_match);

    //void CalcVectQuation(const Eigen::Vector3d& x_vec, const Eigen::Vector3d& y_vec,
    //    const Eigen::Vector3d& z_vec,
    //    geometry_msgs::Quaternion& q) {
    //
    //    Eigen::Matrix3d rot;
    //    rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0),
    //        z_vec(1), z_vec(2);
    //    Eigen::Matrix3d rotation = rot.transpose();
    //    Eigen::Quaterniond eq(rotation);
    //    q.w = eq.w();
    //    q.x = eq.x();
    //    q.y = eq.y();
    //    q.z = eq.z();
    //}

    //void CalcQuation(const Eigen::Vector3d& vec, const int axis,
    //    geometry_msgs::Quaternion& q) {
    //    Eigen::Vector3d x_body = vec;
    //    Eigen::Vector3d y_body(1, 1, 0);
    //    if (x_body(2) != 0) {
    //        y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
    //    }
    //    else {
    //        if (x_body(1) != 0) {
    //            y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
    //        }
    //        else {
    //            y_body(0) = 0;
    //        }
    //    }
    //    y_body.normalize();
    //    Eigen::Vector3d z_body = x_body.cross(y_body);
    //    Eigen::Matrix3d rot;
    //
    //    rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
    //        z_body(0), z_body(1), z_body(2);
    //    Eigen::Matrix3d rotation = rot.transpose();
    //    if (axis == 2) {
    //        Eigen::Matrix3d rot_inc;
    //        rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    //        rotation = rotation * rot_inc;
    //    }
    //    Eigen::Quaterniond eq(rotation);
    //    q.w = eq.w();
    //    q.x = eq.x();
    //    q.y = eq.y();
    //    q.z = eq.z();
    //}

    /*void pubSinglePlane(visualization_msgs::MarkerArray& plane_pub,
        const std::string plane_ns, const Plane& single_plane,
        const float alpha, const Eigen::Vector3d rgb) {
        visualization_msgs::Marker plane;
        plane.header.frame_id = "camera_init";
        plane.header.stamp = ros::Time();
        plane.ns = plane_ns;
        plane.id = single_plane.id;
        plane.type = visualization_msgs::Marker::CYLINDER;
        plane.action = visualization_msgs::Marker::ADD;
        plane.pose.position.x = single_plane.center[0];
        plane.pose.position.y = single_plane.center[1];
        plane.pose.position.z = single_plane.center[2];
        geometry_msgs::Quaternion q;
        CalcVectQuation(single_plane.x_normal, single_plane.y_normal,
            single_plane.normal, q);
        plane.pose.orientation = q;
        plane.scale.x = 3 * sqrt(single_plane.max_eigen_value);
        plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value);
        plane.scale.z = 2 * sqrt(single_plane.min_eigen_value);
        plane.color.a = alpha;
        plane.color.r = rgb(0);
        plane.color.g = rgb(1);
        plane.color.b = rgb(2);
        plane.lifetime = ros::Duration();
        plane_pub.markers.push_back(plane);
    }
    */

    /*void pubNoPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map,
        const ros::Publisher& plane_map_pub) {
        int id = 0;
        ros::Rate loop(500);
        float use_alpha = 0.8;
        visualization_msgs::MarkerArray voxel_plane;
        voxel_plane.markers.reserve(1000000);
        for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
            if (!iter->second->plane_ptr_->is_plane) {
                for (uint i = 0; i < 8; i++) {
                    if (iter->second->leaves_[i] != nullptr) {
                        OctoTree* temp_octo_tree = iter->second->leaves_[i];
                        if (!temp_octo_tree->plane_ptr_->is_plane) {
                            for (uint j = 0; j < 8; j++) {
                                if (temp_octo_tree->leaves_[j] != nullptr) {
                                    if (!temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                                        Eigen::Vector3d plane_rgb(1, 1, 1);
                                        pubSinglePlane(voxel_plane, "no_plane",
                                            *(temp_octo_tree->leaves_[j]->plane_ptr_),
                                            use_alpha, plane_rgb);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        plane_map_pub.publish(voxel_plane);
        loop.sleep();
    }
    */


    /*void pubVoxelMap(const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
        const int pub_max_voxel_layer,
        const ros::Publisher& plane_map_pub) {
        double max_trace = 0.25;
        double pow_num = 0.2;
        ros::Rate loop(500);
        float use_alpha = 0.8;
        visualization_msgs::MarkerArray voxel_plane;
        voxel_plane.markers.reserve(1000000);
        std::vector<Plane> pub_plane_list;
        for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
            GetUpdatePlane(iter->second, pub_max_voxel_layer, pub_plane_list);
        }
        for (size_t i = 0; i < pub_plane_list.size(); i++) {
            V3D plane_cov = pub_plane_list[i].plane_cov.block<3, 3>(0, 0).diagonal();
            double trace = plane_cov.sum();
            if (trace >= max_trace) {
                trace = max_trace;
            }
            trace = trace * (1.0 / max_trace);
            trace = pow(trace, pow_num);
            uint8_t r, g, b;
            mapJet(trace, 0, 1, r, g, b);
            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
            double alpha;
            if (pub_plane_list[i].is_plane) {
                alpha = use_alpha;
            }
            else {
                alpha = 0;
            }
            pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
        }
        plane_map_pub.publish(voxel_plane);
        loop.sleep();
    }

    void pubPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map,
        const ros::Publisher& plane_map_pub) {
        OctoTree* current_octo = nullptr;

        double max_trace = 0.25;
        double pow_num = 0.2;
        ros::Rate loop(500);
        float use_alpha = 1.0;
        visualization_msgs::MarkerArray voxel_plane;
        voxel_plane.markers.reserve(1000000);

        for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
            if (iter->second->plane_ptr_->is_update) {
                Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

                V3D plane_cov =
                    iter->second->plane_ptr_->plane_cov.block<3, 3>(0, 0).diagonal();
                double trace = plane_cov.sum();
                if (trace >= max_trace) {
                    trace = max_trace;
                }
                trace = trace * (1.0 / max_trace);
                trace = pow(trace, pow_num);
                uint8_t r, g, b;
                mapJet(trace, 0, 1, r, g, b);
                Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                // Eigen::Vector3d plane_rgb(1, 0, 0);
                float alpha = 0.0;
                if (iter->second->plane_ptr_->is_plane) {
                    alpha = use_alpha;
                }
                else {
                    // std::cout << "delete plane" << std::endl;
                }
                // if (iter->second->update_enable_) {
                //   plane_rgb << 1, 0, 0;
                // } else {
                //   plane_rgb << 0, 0, 1;
                // }
                pubSinglePlane(voxel_plane, "plane", *(iter->second->plane_ptr_), alpha,
                    plane_rgb);

                iter->second->plane_ptr_->is_update = false;
            }
            else {
                for (uint i = 0; i < 8; i++) {
                    if (iter->second->leaves_[i] != nullptr) {
                        if (iter->second->leaves_[i]->plane_ptr_->is_update) {
                            Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

                            V3D plane_cov = iter->second->leaves_[i]
                                ->plane_ptr_->plane_cov.block<3, 3>(0, 0)
                                .diagonal();
                            double trace = plane_cov.sum();
                            if (trace >= max_trace) {
                                trace = max_trace;
                            }
                            trace = trace * (1.0 / max_trace);
                            // trace = (max_trace - trace) / max_trace;
                            trace = pow(trace, pow_num);
                            uint8_t r, g, b;
                            mapJet(trace, 0, 1, r, g, b);
                            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                            plane_rgb << 0, 1, 0;
                            // fabs(iter->second->leaves_[i]->plane_ptr_->normal[0]),
                            //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[1]),
                            //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[2]);
                            float alpha = 0.0;
                            if (iter->second->leaves_[i]->plane_ptr_->is_plane) {
                                alpha = use_alpha;
                            }
                            else {
                                // std::cout << "delete plane" << std::endl;
                            }
                            pubSinglePlane(voxel_plane, "plane",
                                *(iter->second->leaves_[i]->plane_ptr_), alpha,
                                plane_rgb);
                            // loop.sleep();
                            iter->second->leaves_[i]->plane_ptr_->is_update = false;
                            // loop.sleep();
                        }
                        else {
                            OctoTree* temp_octo_tree = iter->second->leaves_[i];
                            for (uint j = 0; j < 8; j++) {
                                if (temp_octo_tree->leaves_[j] != nullptr) {
                                    if (temp_octo_tree->leaves_[j]->octo_state_ == 0 &&
                                        temp_octo_tree->leaves_[j]->plane_ptr_->is_update) {
                                        if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                                            // std::cout << "subsubplane" << std::endl;
                                            Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);
                                            V3D plane_cov =
                                                temp_octo_tree->leaves_[j]
                                                ->plane_ptr_->plane_cov.block<3, 3>(0, 0)
                                                .diagonal();
                                            double trace = plane_cov.sum();
                                            if (trace >= max_trace) {
                                                trace = max_trace;
                                            }
                                            trace = trace * (1.0 / max_trace);
                                            // trace = (max_trace - trace) / max_trace;
                                            trace = pow(trace, pow_num);
                                            uint8_t r, g, b;
                                            mapJet(trace, 0, 1, r, g, b);
                                            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                                            plane_rgb << 0, 0, 1;
                                            float alpha = 0.0;
                                            if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                                                alpha = use_alpha;
                                            }

                                            pubSinglePlane(voxel_plane, "plane",
                                                *(temp_octo_tree->leaves_[j]->plane_ptr_),
                                                alpha, plane_rgb);
                                            // loop.sleep();
                                            temp_octo_tree->leaves_[j]->plane_ptr_->is_update = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        plane_map_pub.publish(voxel_plane);
        // plane_map_pub.publish(voxel_norm);
        loop.sleep();
        // cout << "[Map Info] Plane counts:" << plane_count
        //      << " Sub Plane counts:" << sub_plane_count
        //      << " Sub Sub Plane counts:" << sub_sub_plane_count << endl;
        // cout << "[Map Info] Update plane counts:" << update_count
        //      << "total size: " << feat_map.size() << endl;
    }
    */
    void calcBodyCov(Eigen::Vector3d& pb, const float range_inc,
        const float degree_inc, Eigen::Matrix3d& cov);


#endif