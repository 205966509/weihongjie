#ifndef CPO_H
#define CPO_H

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <thread>
#include <mutex>
#include <random>
#include "trajectoryGeneration.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>                    // 新增：用于发布路径消息
#include <geometry_msgs/PoseStamped.h>        // 新增：用于定义路径中的姿态信息

using PositionSequence = std::vector<std::vector<std::pair<int, int>>>;

namespace global_motion_planner
{
  struct Particle
  {
    std::vector<std::pair<int, int>> position;               // Particle position
    double fitness;                                          // Particle fitness
    std::vector<std::pair<int, int>> personal_best_pos;      // Personal best position in iteration
    double personal_best_fitness;                            // Personal best fitness in iteration

    Particle() = default;  

    Particle(const std::vector<std::pair<int, int>>& initial_position,
            double initial_fitness)
        : position(initial_position),
          fitness(initial_fitness),
          personal_best_pos(initial_position),
          personal_best_fitness(initial_fitness)
    {
    }
  };

  /**
   * @brief Class for objects that plan using the CPO algorithm
   */
  class CPO 
  {
  public:
    CPO(int nx, int ny, double resolution, double origin_x, double origin_y, int n_particles, int n_inherited, int pointNum , double alpha, double Tf, double N_min, double obs_factor,int T,int initposmode ,bool pub_particles,int max_iter);
    ~CPO();

    bool plan(const unsigned char* global_costmap, const std::pair<int, int>& start, const std::pair<int, int>& goal, std::vector< std::pair<int, int>>& path);
    void generateRandomInitialPositions(PositionSequence &initialPositions, const std::pair<double, double> start_d, const std::pair<double, double> goal_d);
    void generateCircularInitialPositions(PositionSequence &initialPositions, const std::pair<double, double> start_d, const std::pair<double, double> goal_d);
    double ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& cpo_path);
    
    // Updated function signatures to accept second_best_particle
    void updateParticlePosition(Particle& particle, int iter, double& total_fitness, std::vector<Particle>& particles, Particle& Best_particle, Particle& Second_best_particle);
    void optimizeParticle(Particle& particle, Particle& best_particle, Particle& second_best_particle, const unsigned char* global_costmap, const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d, const int& index_i, std::mt19937& gen, int iter, double& total_fitness, std::vector<Particle>& particles);
    
    void handleBoundary(std::pair<int, int>& position, const std::pair<double, double>& Lb, const std::pair<double, double>& Ub, Particle& Best_particle, Particle& Second_best_particle, int index, int t, int Max_iterations, std::mt19937& gen);
    void publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, const int& index);

    int grid2Index (int x, int y);
    static bool ascendingOrder(int a, int b);
    static bool descendingOrder(int a, int b);


  protected:
    int nx_,ny_,ns_;
    double resolution_;
    double origin_x_,origin_y_;
    bool pub_particles_;
    int max_iter_;
    int n_particles_;
    int n_inherited_;
    int pointNum_;
    double alpha_, Tf_, N_min_;
    double obs_factor_;
    int T_;
    int initposmode_;

    // 次优位置
    int SecondBest_particle_;  // 次优索引
    Particle second_best_particle;  

  private:
    unsigned char lethal_cost_=253;
    ros::Publisher particle_pub;
    ros::Publisher path_pub;                     // 路径发布器
    int GlobalBest_particle_;
    std::mutex particles_lock_;
    std::vector<Particle> inherited_particles_;
    trajectoryGeneration path_generation;

    // 成员函数声明
    bool isLineClear(int x0, int y0, int x1, int y1, const unsigned char* global_costmap);                                      // 判断两点之间是否有障碍物
    void simplifyPathBresenham(const unsigned char* global_costmap, std::vector<std::pair<int, int>>& path);                       // 使用Bresenham算法简化路径
    void publishPath(const std::vector<std::pair<int, int>>& path);                                                              // 发布路径到ROS
    void publishKeyPoints(const std::vector<std::pair<int, int>>& key_points);                                                   // 发布关键点到ROS
  };

}  // namespace global_motion_planner

#endif
