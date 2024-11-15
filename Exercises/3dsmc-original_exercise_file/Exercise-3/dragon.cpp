#include "utils/io.h"
#include "utils/points.h"
#include <iostream>
#include "ceres/ceres.h"
#include <math.h>
#include <vector>

// TODO: Implement the cost function (check gaussian.cpp for reference)
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& point_1, const Point2D& point_2, const std::vector<Weight>& weights_)
		: point1(point_1), point2(point_2), weights(weights_)
	{
	}

	template<typename T>
	bool operator()(const T* const angle, const T* const tx, const T* const ty, T* residual) const
	{
		residual[0] = T(0.0);
		// TODO: Implement the cost function
		//residual[0] = point.y - ceres::exp(-(ceres::pow(point.x - mu[0], T(2)) / (T(2) * ceres::pow(sigma[0], T(2)))));;
		//residual[0] = (-c[0] * point.z) + (ceres::pow(point.x, T(2))/a[0]) - (ceres::pow(point.y, T(2))/b[0]);
		 // 对于每个weight 都要计算残差并且求和才是一2dpoint的残差值{from结构体}
		//如果想要auto 自动loop weights实例， 就需要用我自己写的util 里面的point。h中的weigth结构体 毕竟我改成了vector
		for (const auto& weight : weights) {
        	// 遍历 Weight 中的每个权重值
        		for (const auto& single_weight : weight.w) {
            			residual[0] += single_weight *  ceres::pow(
					ceres::pow((ceres::cos(angle[0]) * point1.x - ceres::sin(angle[0]) * point1.y + tx[0] - point2.x), T(2.0)) +
					ceres::pow((ceres::sin(angle[0]) * point1.x + ceres::cos(angle[0]) * point1.y + ty[0] - point2.y), T(2.0))
					, T(0.5));
        		}
    		}

		return true;
	}

private:
	const Point2D point1;
	const Point2D point2;
	const std::vector<Weight> weights;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights); // weights is a  double 数组
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block (check gaussian.cpp for reference)
	for (size_t i = 0; i < points1.size(); ++i)
	{

				problem.AddResidualBlock(
					new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(  // 第一个1 代表残差维度是1
						new RegistrationCostFunction(points1[i], points2[i], weights) 
					// 卧槽 这个b点我得多试几次 我哪里知道谁先谁后呀况且这个weight估计是老师自己已经拟合好的数据所以point12必须有个先后顺序，只能画图试试
					//很可惜 我的matlabTUM证书居然过期了， ，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，，
					),
					nullptr, &angle, &tx, &ty
				);

	}

	ceres::Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}

//  致敬每一位留子
/* 《浪淘沙》李煜           
簾外雨潺潺，春意闌珊。羅衾不耐五更寒，夢裏不知身是客，一晌貪歡。 　
獨自莫凭闌，無限江山，別時容易見時難。流水落花春去也，天上人間！
*/
