#pragma once

#include "internal/PoseD.h"
#include <fstream>
#include <memory>

namespace sc {

class TrajectoryManager {
public:
	typedef std::shared_ptr<TrajectoryManager> Ptr;

bool loadTrajectory(const std::string& path) {
	std::ifstream fIn(path.c_str(), std::ios::in);
	if (!fIn.is_open()) {
		return false;
	}

	trajPoints_.clear();

	std::string line, elem;
	while (std::getline(fIn, line)) {
		std::vector<std::string> elems;
		std::stringstream ss(line);

		while (std::getline(ss, elem, ' ')) {
			elems.push_back(elem);
		}

		if (elems.size() != 9) {
			continue;
		}

		PointPose trajPt;

		trajPt.time = std::stod(elems[0]);
		trajPt.pose.q.w() = std::stod(elems[1]);
		trajPt.pose.q.x() = std::stod(elems[2]);
		trajPt.pose.q.y() = std::stod(elems[3]);
		trajPt.pose.q.z() = std::stod(elems[4]);
		trajPt.pose.p.x() = std::stod(elems[5]);
		trajPt.pose.p.y() = std::stod(elems[6]);
		trajPt.pose.p.z() = std::stod(elems[7]);
		trajPt.frameId = std::stoi(elems[8]);

		trajPoints_.push_back(trajPt);
	}

	return !trajPoints_.empty();
}

bool loadTimeList(const std::string& filePath) {
	std::ifstream fIn(filePath, std::ios::in);
	if (!fIn.is_open()) {
		return false;
	}

	trajPoints_.clear();

	std::string line, elem;
	while (std::getline(fIn, line)) {
		std::vector<std::string> elems;
		std::stringstream ss(line);

		while (std::getline(ss, elem, ',')) {
			elems.push_back(elem);
		}

		if (elems.size() != 5) {
			continue;
		}

		PointPose trajPt;

		trajPt.time = std::stod(elems[4]);
		trajPt.frameId = std::stoi(elems[0]);
		trajPoints_.push_back(trajPt);
	}

	return !trajPoints_.empty();
}

PointPoses& getPoints() { return trajPoints_; }

PointPose slerp(const double time) const {
	PointPose poseOut;
    poseOut.time = time;

    const auto& upperIter = std::upper_bound(trajPoints_.begin(), trajPoints_.end(), poseOut);
	const auto& lowerIter = upperIter - 1;

    if (upperIter == trajPoints_.end()) {
        return trajPoints_.back();
    }    
    if (upperIter == trajPoints_.begin()) {
        return trajPoints_.front();
    }

    double ratio = (time - lowerIter->time) / (upperIter->time - lowerIter->time);
    ratio = std::min(ratio, 1.0);
    ratio = std::max(ratio, 0.0);

	poseOut.pose = lowerIter->pose.Slerp(ratio, upperIter->pose);
	poseOut.frameId = lowerIter->frameId;

    return poseOut;
}

template <typename T>
void slerp2(const T time, Eigen::Matrix<T, 3, 1>& pos, Eigen::Quaternion<T>& qua) const {
    int upperId = 0;
    for (; upperId < trajPoints_.size(); upperId++) {
        const auto& pt = trajPoints_[upperId];
        if (T(pt.time) > time) {
            break;
        }
    }
    int lowerId = upperId - 1;
    const auto upperIter = trajPoints_.begin() + upperId;
    const auto lowerIter = trajPoints_.begin() + lowerId;

    T ratio = (time - T(lowerIter->time)) /
              (T(upperIter->time) - T(lowerIter->time));

    ratio = std::min(ratio, T(1.0));
    ratio = std::max(ratio, T(0.0));

    pos = (T(1) - ratio) * lowerIter->pose.p.cast<T>() + ratio * upperIter->pose.p.cast<T>();
    qua = lowerIter->pose.q.cast<T>().slerp(ratio, upperIter->pose.q.cast<T>());
}

private:
	PointPoses trajPoints_;
};
}
