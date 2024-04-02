#pragma once

#include "internal/Structs.h"
#include "plane_detection/pointcloudio.hpp"
#include "plane_detection/planedetector.h"
#include "plane_detection/normalestimator.h"
#include "plane_detection/boundaryvolumehierarchy.h"
#include "plane_detection/connectivitygraph.h"

#include <pcl/io/ply_io.h>

// namespace of sensor calibration
namespace sc {

// namespace of plane extract
namespace plane_extract {

inline bool ExtractPlaneCloud(const BaseCloudPtr& cloudIn, BaseCloudPtr& cloudOut,
							  const std::vector<std::vector<int>>& indexes) {
	std::vector<Point3d> points;
	points.resize(cloudIn->size());
	BasePoint pt;
	for (int i = 0; i < cloudIn->size(); i++) {
		pt = cloudIn->points[i];
		points[i].position(Eigen::Vector3f(pt.x, pt.y, pt.z));
	}
	PointCloud3d *pointCloud = new PointCloud3d(points);

	ConnectivityGraph *connectivity = new ConnectivityGraph(pointCloud->size());
	pointCloud->connectivity(connectivity);

#pragma omp parallel for num_threads(4)
	for (int i = 0; i < cloudIn->size(); i++) {
		const BasePoint pt = cloudIn->points[i];

		(*pointCloud)[i].normal(pt.getNormalVector3fMap());
		(*pointCloud)[i].normalConfidence(pt.data[3]);
		(*pointCloud)[i].curvature(pt.data_n[3]);
	}

	for (int i = 0; i < cloudIn->size(); i++) {
		connectivity->addNode(i, indexes[i]);
	}

	PlaneDetector detector(pointCloud);
	detector.minNormalDiff(0.5f);
	detector.maxDist(0.25f);
	detector.outlierRatio(0.75f);

	std::set<Plane*> planes = detector.detect();

	// 这个写法不对
	//std::sort(planes.begin(), planes.end(), [](const std::set<Plane*>::iterator l, const std::set<Plane*>::iterator r)
	//										{return (*l)->inliers().size() < (*r)->inliers().size(); });

	std::cout << "Plane detect cout: " << planes.size() << std::endl;

	//std::string debugPath = "E:\\Projects\\git_code\\lidar_imu_calib\\2022-12-01-13-47-10\\raw_data\\";
	//int countId = 0;
	cloudOut->clear();
	for (Plane *plane : planes) {
		std::vector<size_t> inliersIndex = plane->inliers();
		BaseCloud curPlane;

		for (const size_t& id : inliersIndex) {
			curPlane.push_back(cloudIn->points[id]);
		}

		*cloudOut += curPlane;
		//pcl::io::savePLYFileBinary(debugPath + std::to_string(countId) + "_source_plane_segment.ply", curPlane);
		//countId++;
	}

	return !cloudOut->empty();
}
}

}// namespace sc
