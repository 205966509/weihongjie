#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <vector>
#include <utility>

class trajectoryGeneration
{
    public:
        trajectoryGeneration(); // 构造函数
        ~trajectoryGeneration(); // 析构函数

        static int splineOrder; // 设置B样条曲线的阶数

        /**
         * @brief 使用准均匀B样条曲线进行轨迹生成
         * @param plan    关键点&生成的轨迹
         * @param k       B样条曲线的阶数
         */
        void B_spline_curve(std::vector<std::pair<double, double>> &plan, int k);


        /**
         * @brief 生成B样条曲线控制点
         * @param start_d           起始点
         * @param goal_d            目标点
         * @param initial_position  中间控制点
         * @param initial_point     B样条控制点
         */
        void GenerateControlPoints(const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d,
                            const std::vector<std::pair<int, int>>& initial_position,
                            std::vector<std::pair<double, double>>& initial_point);

        /**
         * @brief 计算两点之间的距离
         * @param point1     第一个点
         * @param point2     第二个点
         * @return 两点之间的距离
         */
        double calculateDistance(const std::pair<double, double>& point1, const std::pair<double, double>& point2);

        /**
         * @brief 计算路径长度
         * @param path  要计算的路径
         * @return 路径长度
         */
        double calculatePathLength(const std::vector<std::pair<double, double>>& path);


    private:
        double BaseFun(int i, int k, double u, std::vector<double> NodeVector); // 基函数计算
};

#endif // TRAJECTORY_GENERATION_H