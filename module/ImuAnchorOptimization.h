#ifndef __IMUANCHOROPTIMIZATION_H__
#define __IMUANCHOROPTIMIZATION_H__
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <internal/PoseD.h>
namespace sc {


using Vec7d = Eigen::Matrix<double, 7, 1>;

class ImuAnchorOptimization {
public:
	ImuAnchorOptimization() {};

	~ImuAnchorOptimization() {};

	bool Run();

//private:
	// ������̼�����
	bool InputOdo(Eigen::Quaterniond &q, Eigen::Vector3d &t);

	// ����ê������
	bool InputAnchor(Eigen::Vector3d &t);
	
	//�Ż�
	bool optimize();

	


	

private:
	
	std::vector<PoseD> Local_pos_;
	std::vector<Eigen::Vector3d> Anchor_pos_;
};

}// namespace sc
#endif // !__IMUANCHOROPTIMIZATION_H__
