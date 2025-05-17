#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "cpo.h"


namespace global_motion_planner
{
  CPO::CPO(int nx, int ny, double resolution, double origin_x, double origin_y,int n_particles,int n_inherited,int pointNum , double alpha, double Tf, double N_min, double obs_factor ,int T, int initposmode, bool pub_particles,int max_iter)
    : nx_(nx)
    , ny_(ny)
    , ns_(nx*ny)
    , resolution_(resolution)
    , origin_x_(origin_x)
    , origin_y_(origin_y)
    , n_particles_(n_particles)
    , n_inherited_(n_inherited)
    , pointNum_(pointNum)
    , alpha_(alpha)
    , Tf_(Tf)
    , N_min_(N_min)
    , obs_factor_(obs_factor)
    , T_(T)
    , initposmode_(initposmode)
    , pub_particles_(pub_particles)
    , max_iter_(max_iter)
  {
 inherited_particles_.emplace_back(std::vector<std::pair<int, int>>(pointNum, std::make_pair(1, 1)),
                               0.0);
  // Initialize ROS publisher
  ros::NodeHandle nh;
  particle_pub = nh.advertise<visualization_msgs::Marker>("particle_swarm_markers", 10);
  path_pub = nh.advertise<nav_msgs::Path>("planned_path", 10); // 添加初始化
  }

  CPO::~CPO()
  {
  }
bool CPO::plan(const unsigned char* global_costmap, const std::pair<int, int>& start, const std::pair<int, int>& goal, std::vector<std::pair<int, int>>& path)
{
    std::cout << "CPO planning started..." << std::endl;

    // Variable initialization
    double pathLength;
    double initial_fitness;
    double obstacle_cost;
    Particle Best_particle;
    PositionSequence initialPositions;
    std::vector<Particle> particles;
    std::vector<std::pair<double, double>> initial_point;
    std::pair<double, double> start_d(static_cast<double>(start.first), static_cast<double>(start.second));
    std::pair<double, double> goal_d(static_cast<double>(goal.first), static_cast<double>(goal.second));

    // Generate initial position of particle swarm
    if (initposmode_ == 1) {
        generateRandomInitialPositions(initialPositions, start_d, goal_d);
    } else {
        generateCircularInitialPositions(initialPositions, start_d, goal_d);
    }

    std::cout << "CPO: Successfully generated initial position of particle swarm" << std::endl;

    // Particle initialization
    for (int i = 0; i < n_particles_; ++i) {
        std::vector<std::pair<int, int>> initial_position;

        if ((i < n_inherited_) && (inherited_particles_.size() == n_inherited_)) {
            initial_position = inherited_particles_[i].personal_best_pos;
        } else {
            initial_position = initialPositions[i];
        }

        path_generation.GenerateControlPoints(start_d, goal_d, initial_position, initial_point);
        path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
        pathLength = path_generation.calculatePathLength(initial_point);
        obstacle_cost = ObstacleCost(global_costmap, initial_point);
        initial_fitness = 100000.0 / (pathLength + 1000 * obstacle_cost);

        // Update the best particle and second-best particle
        if (i == 0) {
            GlobalBest_particle_ = i;
            Best_particle.fitness = initial_fitness;
            Best_particle.position = initial_position;
        } else if (initial_fitness > Best_particle.fitness) {
            SecondBest_particle_ = GlobalBest_particle_;  // Current best becomes second-best
            second_best_particle = Best_particle;         // Backup best to second-best
            GlobalBest_particle_ = i;                     // Update best particle
            Best_particle.fitness = initial_fitness;
            Best_particle.position = initial_position;
        } else if (i == 1 || initial_fitness > second_best_particle.fitness) {
            SecondBest_particle_ = i;  // Update second-best particle
            second_best_particle.fitness = initial_fitness;
            second_best_particle.position = initial_position;
        }

        particles.emplace_back(initial_position, initial_fitness);
    }

    Best_particle.position = particles[GlobalBest_particle_].position;

    std::cout << "CPO: Successfully generated initial particle swarm" << std::endl;

    // Random data
    std::random_device rd;
    std::mt19937 gen(rd());

    std::cout << "CPO: Particle swarm iteration progress: " << std::endl;

    for (size_t iter = 0; iter < max_iter_; iter++) {
        std::cout << "CPO " << iter << " ";

        double total_fitness = 0.0;

        // 创建一个局部变量，避免线程安全问题
        int current_n_particles = n_particles_;
        std::vector<std::thread> particle_list(current_n_particles);

        for (size_t i = 0; i < current_n_particles; ++i) {
            particle_list[i] = std::thread(&CPO::optimizeParticle, this,
                                           std::ref(particles[i]), std::ref(Best_particle), std::ref(second_best_particle),
                                           std::cref(global_costmap), std::cref(start_d),
                                           std::cref(goal_d), i, std::ref(gen), iter,
                                           std::ref(total_fitness), std::ref(particles));
        }

        for (size_t i = 0; i < current_n_particles; ++i) {
            particle_list[i].join();
        }

        // 动态调整种群规模
        int new_n_particles = static_cast<int>(N_min_ + (n_particles_ - N_min_) * (1 - (iter % (max_iter_ / T_)) / (max_iter_ / T_)));

        if (new_n_particles < n_particles_) {
            // 保留主种群中适应度值最高的个体
            std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) { return a.fitness > b.fitness; });

            // 保存被移除的粒子
            std::vector<Particle> removed_particles(particles.begin() + new_n_particles, particles.end());

            // 缩小主种群
            particles.resize(new_n_particles);
            n_particles_ = new_n_particles;

            // 对被移除的粒子进行差分变异，并注入主种群
            double F_stars = 0.9;
            double F_end = 0.4;
            double F = F_stars - ((F_stars - F_end) * iter / max_iter_);

            std::uniform_int_distribution<int> uniform_int_dist(0, n_particles_ - 1);
            std::uniform_real_distribution<double> distribution(0.0, 1.0);

            for (size_t i = 0; i < removed_particles.size(); ++i) {
                int rand_index1 = uniform_int_dist(gen);
                int rand_index2 = uniform_int_dist(gen);

                while (rand_index2 == rand_index1) {
                    rand_index2 = uniform_int_dist(gen);
                }

                // 差分变异过程
                Particle new_particle;
                new_particle.position.resize(Best_particle.position.size());
                for (size_t d = 0; d < Best_particle.position.size(); ++d) {
                    int diff_x = particles[rand_index1].position[d].first - particles[rand_index2].position[d].first;
                    int diff_y = particles[rand_index1].position[d].second - particles[rand_index2].position[d].second;

                    double perturbation_x = F * diff_x;
                    double perturbation_y = F * diff_y;

                    // 在最优解附近生成新粒子
                    double new_x = Best_particle.position[d].first + perturbation_x;
                    double new_y = Best_particle.position[d].second + perturbation_y;

                    // 边界处理
                    new_x = std::max(std::min(new_x, static_cast<double>(nx_ - 1)), 0.0);
                    new_y = std::max(std::min(new_y, static_cast<double>(ny_ - 1)), 0.0);

                    new_particle.position[d] = std::make_pair(static_cast<int>(std::round(new_x)), static_cast<int>(std::round(new_y)));
                }

                // 计算新粒子的适应度值
                std::vector<std::pair<double, double>> new_path;
                path_generation.GenerateControlPoints(start_d, goal_d, new_particle.position, new_path);
                path_generation.B_spline_curve(new_path, path_generation.splineOrder);
                double pathLength = path_generation.calculatePathLength(new_path);
                double obstacle_cost = ObstacleCost(global_costmap, new_path);
                new_particle.fitness = 100000.0 / (pathLength + 1000 * obstacle_cost);

                // 更新个人最优
                new_particle.personal_best_fitness = new_particle.fitness;
                new_particle.personal_best_pos = new_particle.position;

                // 更新全局最优解
                {
                    std::lock_guard<std::mutex> lock(particles_lock_);
                    if (new_particle.fitness > Best_particle.fitness) {
                        second_best_particle = Best_particle;
                        Best_particle = new_particle;
                        GlobalBest_particle_ = particles.size();  // 新粒子的索引
                    }
                }

                // 将新粒子注入主种群
                particles.push_back(new_particle);
            }

            // 更新种群数量
            n_particles_ = particles.size();
        }

        // 计算所有粒子的总适应度
        total_fitness = 0.0;
        for (const auto& particle : particles) {
            total_fitness += particle.fitness;
        }

        Best_particle.position = particles[GlobalBest_particle_].personal_best_pos;
    }

    // Generating Paths from Optimal Particles
    path_generation.GenerateControlPoints(start_d, goal_d, Best_particle.position, initial_point);
    path_generation.B_spline_curve(initial_point, path_generation.splineOrder);

    std::cout << "CPO: Iteration completed, optimal fitness is: " << Best_particle.fitness << std::endl;

    // Path data structure conversion
    path.clear();

    if (!initial_point.empty()) {
        // Add the first point
        path.emplace_back(static_cast<int>(initial_point[0].first), static_cast<int>(initial_point[0].second));

        for (size_t p = 1; p < initial_point.size(); ++p) {
            int x = static_cast<int>(initial_point[p].first);
            int y = static_cast<int>(initial_point[p].second);
            // Check if the current point is different from the last point
            if (x != path.back().first || y != path.back().second) {
                path.emplace_back(x, y);
            }
        }
    }
    simplifyPathBresenham(global_costmap, path);
    std::vector<std::pair<int, int>> key_points = path;

    // Update inheritance particles based on optimal fitness
    std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) { return a.personal_best_fitness > b.personal_best_fitness; });
    inherited_particles_.clear();

    for (size_t inherit = 0; inherit < n_inherited_; ++inherit) {
        inherited_particles_.emplace_back(particles[inherit]);
    }
    if (!key_points.empty()) {
        std::cout << "CPO: Planning Successful!" << std::endl;
        publishPath(key_points); // 发布关键路径
        publishKeyPoints(key_points); // 发布关键点
    }

    return !key_points.empty();

}

  // Transform from grid map(x, y) to grid index(i)
  int CPO::grid2Index(int x, int y){return x + nx_ * y;}


  void CPO::generateRandomInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d)
  {
      // Use a random device and engine to generate random numbers
      std::random_device rd;
      std::mt19937 gen(rd());
      int x[pointNum_], y[pointNum_];
      int point_id;

      //Calculate sequence direction
      bool xorder = (goal_d.first > start_d.first);
      bool yorder = (goal_d.second > start_d.second);

      for (int i = 0; i < n_particles_; ++i)
      {
          std::unordered_set<int> visited;
          std::vector<std::pair<int, int>> particlePositions;
          point_id=0;
          // Generate pointNum_ unique coordinates
          while (point_id < pointNum_)
          {
              x[point_id] = std::uniform_int_distribution<int>(0, nx_-1)(gen);
              y[point_id] = std::uniform_int_distribution<int>(0, ny_-1)(gen);
              int uniqueId = x[point_id] * (ny_ + 1) + y[point_id];  // Represent coordinates by a unique ID

              // Check if the coordinates have already been used
              if (visited.find(uniqueId) == visited.end())
              {   
                  point_id=point_id+1;
                  visited.insert(uniqueId);
              }
          }

          //sort
          if(xorder){std::sort(x, x + pointNum_, &CPO::ascendingOrder);}
          else{std::sort(x, x + pointNum_, &CPO::descendingOrder);}

          if(yorder){std::sort(y, y + pointNum_, &CPO::ascendingOrder);}
          else{std::sort(y, y + pointNum_, &CPO::descendingOrder);}

          // Store elements from x and y in particlePositions
          for (int ii = 0; ii < pointNum_; ++ii)
          {
              particlePositions.emplace_back(x[ii], y[ii]);
          }

          initialPositions.push_back(particlePositions);
      }
  }
  bool CPO::ascendingOrder(int a, int b) {
    return a < b;
}

  bool CPO::descendingOrder(int a, int b) {
    return a > b;
}



  void CPO::generateCircularInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d)
  {
      // Use a random device and engine to generate random numbers
      std::random_device rd;
      std::mt19937 gen(rd());
      int x[pointNum_], y[pointNum_];
      int point_id;
      //Calculate sequence direction
      bool xorder = (goal_d.first > start_d.first);
      bool yorder = (goal_d.second > start_d.second);
      // Calculate the center of the circle (midpoint between start and goal)
      int centerX = (start_d.first + goal_d.first) / 2;
      int centerY = (start_d.second + goal_d.second) / 2;
      // Calculate the radius of the circle (half of the distance between start and goal)
      double radius = path_generation.calculateDistance(start_d,goal_d) / 2.0;

      if (radius<5){radius=5;}

      for (int i = 0; i < n_particles_; ++i)
      {
          std::unordered_set<int> visited;
          std::vector<std::pair<int, int>> particlePositions;
          point_id=0;
          // Generate pointNum_ unique coordinates
          while (point_id < pointNum_)
          {  
            // Generate random angle in radians
            double angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(gen);
            // Generate random distance from the center within the circle
            double r = std::sqrt(std::uniform_real_distribution<double>(0, 1)(gen)) * radius;
            // Convert polar coordinates to Cartesian coordinates
            x[point_id] = static_cast<int>(std::round(centerX + r * std::cos(angle)));
            y[point_id] = static_cast<int>(std::round(centerY + r * std::sin(angle)));

            // Check if the coordinates are within the map range
            if (x[point_id] >= 0 && x[point_id] < nx_ && y[point_id] >= 0 && y[point_id] < ny_) 
            {
                int uniqueId = x[point_id] * (ny_ + 1) + y[point_id];
                // Check if the coordinates have already been used
                if (visited.find(uniqueId) == visited.end()) 
                {
                    point_id = point_id + 1;
                    visited.insert(uniqueId);
                }
            }

          }

          //sort
          if(xorder){std::sort(x, x + pointNum_, &CPO::ascendingOrder);}
          else{std::sort(x, x + pointNum_, &CPO::descendingOrder);}

          if(yorder){std::sort(y, y + pointNum_, &CPO::ascendingOrder);}
          else{std::sort(y, y + pointNum_, &CPO::descendingOrder);}

          // 将 x 和 y 中的元素存放到 particlePositions 中
          for (int ii = 0; ii < pointNum_; ++ii)
          {
              particlePositions.emplace_back(x[ii], y[ii]);
          }

          initialPositions.push_back(particlePositions);
      }
  }

  double CPO::ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& cpo_path)
  {
    int point_index;
    double Obscost=1;

    for (size_t i = 1; i < cpo_path.size(); ++i) 
    {
      point_index=grid2Index(static_cast<int>(cpo_path[i].first),  static_cast<int>(cpo_path[i].second));
      // next node hit the boundary or obstacle
      if ((point_index < 0) || (point_index >= ns_) || (global_costmap[point_index] >= lethal_cost_ * obs_factor_))
      {
        Obscost=Obscost+1;
      }
    }

    return Obscost;

  }
  // 判断两点之间是否有障碍物
bool CPO::isLineClear(int x0, int y0, int x1, int y1, const unsigned char* global_costmap) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    
    int x = x0;
    int y = y0;
    
    int n = 1 + dx + dy;
    
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    
    for (; n > 0; --n) {
        int index = grid2Index(x, y);
        if (index < 0 || index >= ns_) {
            return false; // 超出地图边界，视为有障碍
        }
        if (global_costmap[index] >= lethal_cost_ * obs_factor_) {
            return false; // 遇到障碍物
        }
        
        if (error > 0) {
            x += x_inc;
            error -= dy;
        }
        else {
            y += y_inc;
            error += dx;
        }
    }
    
    return true; // 线路无障碍
}
void CPO::simplifyPathBresenham(const unsigned char* global_costmap, std::vector<std::pair<int, int>>& path) {
    if (path.empty()) {
        return;
    }
    
    std::vector<std::pair<int, int>> simplified_path;
    simplified_path.emplace_back(path[0]); // 保留起点
    
    size_t current = 0;
    size_t next = 1;
    
    while (next < path.size()) {
        // 尝试将当前点与下一个点直接连接
        bool clear = isLineClear(path[current].first, path[current].second, path[next].first, path[next].second, global_costmap);
        
        if (clear) {
            // 如果直线路径无障碍，尝试连接更远的点
            next++;
        }
        else {
            // 如果有障碍，保留前一个点作为新的起点
            simplified_path.emplace_back(path[next - 1]);
            current = next - 1;
        }
    }
    
    // 保留终点
    if (simplified_path.back() != path.back()) {
        simplified_path.emplace_back(path.back());
    }
    
    path = simplified_path;
}
void CPO::handleBoundary(std::pair<int, int>& position, const std::pair<double, double>& Lb, const std::pair<double, double>& Ub, 
                         Particle& Best_particle, Particle& Second_best_particle, int index, int t, int Max_iterations, std::mt19937& gen) {
    double transition = 1.0 - static_cast<double>(t) / Max_iterations;
    std::uniform_real_distribution<> dist(0.0, 1.0);

    // 将 position 转换为 double 类型以进行处理
    double pos_x = static_cast<double>(position.first);
    double pos_y = static_cast<double>(position.second);

    if (pos_x < Lb.first || pos_x > Ub.first || pos_y < Lb.second || pos_y > Ub.second) {
        if (transition > dist(gen)) {
            // 在边界内重新初始化
            pos_x = Lb.first + dist(gen) * (Ub.first - Lb.first);
            pos_y = Lb.second + dist(gen) * (Ub.second - Lb.second);
        } else {
            // 基于最佳粒子和次优粒子的位置进行调整
            pos_x = Second_best_particle.position[index].first + 
                    dist(gen) * (Best_particle.position[index].first - Second_best_particle.position[index].first);
            pos_y = Second_best_particle.position[index].second + 
                    dist(gen) * (Best_particle.position[index].second - Second_best_particle.position[index].second);
            // 确保新位置在边界内
            pos_x = std::max(std::min(pos_x, Ub.first), Lb.first);
            pos_y = std::max(std::min(pos_y, Ub.second), Lb.second);
        }
    }

    // 将处理后的 double 类型位置转换回 int 类型
    position.first = static_cast<int>(std::round(pos_x));
    position.second = static_cast<int>(std::round(pos_y));
}
void CPO::updateParticlePosition(Particle& particle, int iter, double& total_fitness, 
                                 std::vector<Particle>& particles, Particle& best_particle, 
                                 Particle& second_best_particle) {
    int dimensions = particle.position.size(); 

    std::random_device rd;
    std::mt19937 gen(rd()); 
    std::uniform_real_distribution<> distribution(0.0, 1.0); 
    std::normal_distribution<double> normal_dist(0.0, 1.0); 
    std::uniform_int_distribution<int> uniform_int_dist(0, n_particles_ - 1);

    double Yt = 2.0 * distribution(gen) * pow(1.0 - static_cast<double>(iter) / max_iter_, static_cast<double>(iter) / max_iter_);
    double r2 = distribution(gen);

  
    std::vector<bool> U1(dimensions);
    std::vector<bool> U2(dimensions);

    for (int i = 0; i < dimensions; ++i) {
       
        U1[i] = distribution(gen) > 0.5;
        U2[i] = distribution(gen) > 0.5;

        double rand_val = distribution(gen);
        
        if (rand_val < rand_val) { 
            if (distribution(gen) < distribution(gen)) { 
                int rand_index = uniform_int_dist(gen);
                auto y = std::make_pair(
                    (particle.position[i].first + particles[rand_index].position[i].first) / 2,
                    (particle.position[i].second + particles[rand_index].position[i].second) / 2
                );
                particle.position[i].first += normal_dist(gen) * abs(2 * best_particle.position[i].first - y.first);
                particle.position[i].second += normal_dist(gen) * abs(2 * best_particle.position[i].second - y.second);
            } else { 
                int rand_index1 = uniform_int_dist(gen);
                int rand_index2 = uniform_int_dist(gen);
                auto y = std::make_pair(
                    (particle.position[i].first + particles[rand_index1].position[i].first) / 2,
                    (particle.position[i].second + particles[rand_index1].position[i].second) / 2
                );
                particle.position[i].first = U1[i] * particle.position[i].first + 
                    (1 - U1[i]) * (y.first + distribution(gen) * 
                    (particles[rand_index1].position[i].first - particles[rand_index2].position[i].first));
                particle.position[i].second = U1[i] * particle.position[i].second + 
                    (1 - U1[i]) * (y.second + distribution(gen) * 
                    (particles[rand_index1].position[i].second - particles[rand_index2].position[i].second));
            }
        } else { 
            double S = distribution(gen) * (distribution(gen) < 0.5);
            if (distribution(gen) < Tf_) { 
                double St = exp(particle.fitness / (total_fitness + 1e-10));
                S *= Yt * St;
                int rand_index1 = uniform_int_dist(gen);
                int rand_index2 = uniform_int_dist(gen);
                int rand_index3 = uniform_int_dist(gen);
                particle.position[i].first = (1 - U1[i]) * particle.position[i].first + 
                    U1[i] * (particles[rand_index1].position[i].first + 
                    St * (particles[rand_index2].position[i].first - particles[rand_index3].position[i].first)) - S;
                particle.position[i].second = (1 - U1[i]) * particle.position[i].second + 
                    U1[i] * (particles[rand_index1].position[i].second + 
                    St * (particles[rand_index2].position[i].second - particles[rand_index3].position[i].second)) - S;
            } else { 
                double Mt = exp(particle.fitness / (total_fitness + 1e-10));
                auto vt = particle.position;
                int rand_index = uniform_int_dist(gen);
                auto Vtp = particles[rand_index].position;
                double Ft = distribution(gen) * (Mt * (-(vt[i].first) + Vtp[i].first));
                S *= Yt * Ft;
                particle.position[i].first = (best_particle.position[i].first + 
                    (alpha_ * (1 - r2) + r2) * 
                    (U2[i] * best_particle.position[i].first - particle.position[i].first)) - S;
                particle.position[i].second = (best_particle.position[i].second + 
                    (alpha_ * (1 - r2) + r2) * 
                    (U2[i] * best_particle.position[i].second - particle.position[i].second)) - S;
            }
        }

        std::pair<double, double> Lb(1.0, 1.0);
        std::pair<double, double> Ub(static_cast<double>(nx_ - 1), static_cast<double>(ny_ - 1));
        handleBoundary(particle.position[i], Lb, Ub, best_particle, second_best_particle, i, iter, max_iter_, gen);
    }
}
 void CPO::optimizeParticle(Particle& particle, Particle& best_particle, Particle& second_best_particle, 
                           const unsigned char* global_costmap, const std::pair<double, double>& start_d, 
                           const std::pair<double, double>& goal_d, const int& index_i, 
                           std::mt19937& gen, int iter, double& total_fitness, 
                           std::vector<Particle>& particles)
{
    std::vector<std::pair<double, double>> process_path;

    // Update particle positions based on best and second-best particles
    updateParticlePosition(particle, iter, total_fitness, particles, best_particle, second_best_particle);

    // Generate B-spline curve control points
    path_generation.GenerateControlPoints(start_d, goal_d, particle.position, process_path);
    path_generation.B_spline_curve(process_path, path_generation.splineOrder);

    // Calculate path length and obstacle avoidance cost
    double pathLength = path_generation.calculatePathLength(process_path);
    double obstacle_cost = ObstacleCost(global_costmap, process_path);

    // Calculate the fitness
    particle.fitness = 100000.0 / (pathLength + 1000 * obstacle_cost);

    // Update individual best position
    if (particle.fitness > particle.personal_best_fitness) {
        particle.personal_best_fitness = particle.fitness;
        particle.personal_best_pos = particle.position;
    }

    // Lock to update global best and second-best particles safely
    particles_lock_.lock();
    if (particle.personal_best_fitness > best_particle.fitness) {
        SecondBest_particle_ = GlobalBest_particle_;  // The current best becomes second-best
        second_best_particle = best_particle;  // Update second-best particle
        best_particle = particle;  // Update best particle
        GlobalBest_particle_ = index_i;
    } 
    particles_lock_.unlock();

    // Publish particle markers if needed
    if (pub_particles_) {
        publishParticleMarkers(particle.position, index_i);
    }
}


   void CPO::publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, const int& index) 
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "particle_swarm";
    marker.id = index;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // Convert particle positions to geometry_msgs::Point
    for (const auto& position : positions) {
      geometry_msgs::Point p;
      p.x = origin_x_ + position.first * resolution_;
      p.y = origin_y_ + position.second * resolution_;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    // Set the lifetime of the marker (e.g., 1 second)
    marker.lifetime = ros::Duration(1.0);

    particle_pub.publish(marker);
  }
   void CPO::publishPath(const std::vector<std::pair<int, int>>& path)
    {
        nav_msgs::Path ros_path;
        ros_path.header.frame_id = "map";
        ros_path.header.stamp = ros::Time::now();

        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = origin_x_ + point.first * resolution_;
            pose.pose.position.y = origin_y_ + point.second * resolution_;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            ros_path.poses.push_back(pose);
        }

        path_pub.publish(ros_path);
    }

    // 实现发布关键点的函数
    void CPO::publishKeyPoints(const std::vector<std::pair<int, int>>& key_points)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "key_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // 完全不透明

        for (const auto& point : key_points) {
            geometry_msgs::Point p;
            p.x = origin_x_ + point.first * resolution_;
            p.y = origin_y_ + point.second * resolution_;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker.lifetime = ros::Duration(1.0);

        particle_pub.publish(marker);
    }
}  // namespace global_planner