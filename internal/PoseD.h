#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

// namespace of sensor calibration
namespace sc {

struct PoseD {
	Eigen::Vector3d p;
	Eigen::Quaterniond q;
	
	PoseD() : p(0, 0, 0), q(1, 0, 0, 0) {}

	void SetQuaternionRPY(const Eigen::Quaterniond& qua) {
		this->q = qua;
	}

	void SetPosePQ(const Eigen::Vector3d& pos, const Eigen::Quaterniond& qua) {
		this->p = pos;
		this->q = qua;
	}

	void SetInverse() {
		this->q = this->q.inverse();
		this->p = -(this->q * this->p);
	}

	PoseD operator*(const PoseD& other) const {
		PoseD poseOut;
		poseOut.p = this->q * other.p + this->p;
		poseOut.q = this->q * other.q;
		return poseOut;
	}

	PoseD Inverse() const {
		PoseD poseOut;
		poseOut.q = this->q.inverse();
		poseOut.p = -(poseOut.q * this->p);
		return poseOut;
	}

	PoseD Slerp(const double ratio, const PoseD& other) const {
		PoseD poseOut;
		poseOut.p = (1 - ratio) * this->p + ratio * other.p;
		poseOut.q = this->q.slerp(ratio, other.q);
		return poseOut;
	}

	void GetXYZRPY(double* pose) const {
		Eigen::Matrix3d rotMat = this->q.toRotationMatrix();
		pose[0] = this->p[0];
		pose[1] = this->p[1];
		pose[2] = this->p[2];
		pose[3] = std::atan2(rotMat(2, 1), rotMat(2, 2));
		pose[4] = std::asin(-rotMat(2, 0));
		pose[5] = std::atan2(rotMat(1, 0), rotMat(0, 0));
	}
};

struct PointPose {
	int frameId;
	double time;
	PoseD pose;

	PointPose() : time(0.),
				  frameId(0){}

	bool operator<(const PointPose other) const {
		return this->time < other.time;
	}
};
typedef std::vector<PointPose> PointPoses;

}// namespace sc