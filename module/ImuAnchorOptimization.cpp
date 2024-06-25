#include "module/ImuAnchorOptimization.h"
#include <internal/CeresFactors.h>
namespace sc {
	bool ImuAnchorOptimization::InputOdo(Eigen::Quaterniond& q, Eigen::Vector3d& t)
	{
		PoseD pd;
		pd.q = q;
		pd.p = t;
		Local_pos_.push_back(pd);
		return true;
	}
	bool ImuAnchorOptimization::InputAnchor(Eigen::Vector3d& t)
	{
		Anchor_pos_.push_back(t);
		return true;
	}
	bool ImuAnchorOptimization::optimize()
	{
		
		ceres::Problem problem;
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		options.minimizer_progress_to_stdout = true;
		options.max_num_iterations = 5;
		ceres::Solver::Summary summary;
		ceres::LossFunction* loss_function;
		loss_function = new ceres::HuberLoss(1.0);
		ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

		int length = Local_pos_.size();
		double** t_array = new double* [length];
		double** q_array = new double* [length];
		for (int i = 0; i < length; i++)
		{
			t_array[i] = new double[3];
			q_array[i] = new double[4];
		}
		for (int i = 0; i < length; i++)
		{
			t_array[i][0] = Local_pos_[i].p[0];
			t_array[i][1] = Local_pos_[i].p[1];
			t_array[i][2] = Local_pos_[i].p[2];

			q_array[i][0] = Local_pos_[i].q.w();
			q_array[i][1] = Local_pos_[i].q.x();
			q_array[i][2] = Local_pos_[i].q.y();
			q_array[i][3] = Local_pos_[i].q.z();

			problem.AddParameterBlock(q_array[i], 4, local_parameterization);
			problem.AddParameterBlock(t_array[i], 3);
		}
		std::vector<PoseD>::iterator iterOdo, iterOdoNext;
		int i;
		for (i = 0; i < length; i++)
		{
			if (i < length - 1)
			{
				PoseD Tij = Local_pos_[i].Inverse() * Local_pos_[i + 1];
				ceres::CostFunction* ImuOdo_function = RelativeRTError::Create(Tij.p[0], Tij.p[1], Tij.p[2], Tij.q.w(), Tij.q.x(), Tij.q.y(), Tij.q.z(), 1, 1);
				problem.AddResidualBlock(ImuOdo_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
			}
			
		}
		ceres::CostFunction* anchor_function = TError::Create(Anchor_pos_[0][0], Anchor_pos_[0][1], Anchor_pos_[0][2],0.1);
		problem.AddResidualBlock(anchor_function, loss_function, t_array[length - 1]);
		problem.SetParameterBlockConstant(q_array[0]);
		problem.SetParameterBlockConstant(t_array[0]);
		//problem.SetParameterBlockConstant(q_array[length-1]);
		//problem.SetParameterBlockConstant(t_array[length -1]);
		ceres::Solve(options, &problem, &summary);
		char strFilePath[125] = { 0 };
		std::sprintf(strFilePath, "C:\\Users\\admin\\Desktop\\share\\pos1.txt");
		FILE* fPos1 = fopen(strFilePath, "w");
		for (int i = 0; i < length; i++)
			fprintf(fPos1, "%16.5f %16.5f %16.5f %16.5f %16.5f %16.5f %16.5f\n", q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3], t_array[i][0], t_array[i][1], t_array[i][2]);
		fclose(fPos1);
		return true;
	}
}// namespace sc
