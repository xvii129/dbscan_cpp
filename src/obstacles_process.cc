/******************************************************************************
 * Name: obstacles process source file
 * Copyright: xxx
 * Date: 2022.10.01
 * Description: obstacles process
 *****************************************************************************/

#include "obstacles_process.h"
#include "dbscan/pedestrian_cluster.h"

#include <iostream>
#include <vector>

namespace cluster {

ObstaclesProcessor::ObstaclesProcessor() {
    movable_obstacle_ids_.clear();
    pedestrian_cluested_ids_.clear();
}

const std::vector<Obstacle>& ObstaclesProcessor::movable_obstacles() const
{
    return movable_obstacle_ids_;
}

const std::unordered_map<int, std::vector<int>>& ObstaclesProcessor::pedestrian_cluested_ids() const
{
    return pedestrian_cluested_ids_;
}

void ObstaclesProcessor::Init()
{
    // 添加10个VEHICLE目标

    // 添加10个CYCLIST目标

    // 添加xxx个PEDESTRIAN目标, 可以选择载入实际数据
    std::vector<Point2d> position = {
        {1.1, 1.2},
        {1.2, 1.1},
        {1.3, 1.4},
        {3.1, 3.2},
        {3.2, 3.1},
        {3.3, 3.4},
        {3.5, 3.4},
        {7.1, 7.2},
        {7.2, 7.3},
        {7.3, 7.4}
    };
    for (int cnt = 0; cnt < 10; cnt++) {
        Obstacle target;
        target.id = cnt;
        target.type = Type::PEDESTRIAN;
        target.pose = position[cnt];
        movable_obstacle_ids_.emplace_back(std::move(target));
    }
    return;
}

void ObstaclesProcessor::DenseCrowdAnalysis()
{
    if (!g_pedestrian_cluster_switch) {
        return;
    }
    std::vector<std::pair<int, Point2d>> pedestrian_feature;
    for (const auto& obstacle : movable_obstacle_ids_) {
        if (obstacle.type != Type::PEDESTRIAN) {
            continue;
        }
        std::pair<int, Point2d> pedestrian_item = std::make_pair(obstacle.id, obstacle.pose);
        pedestrian_feature.emplace_back(std::move(pedestrian_item));
    }
    ClusterAnalysis cluster_analysis;
    cluster_analysis.Init(pedestrian_feature, 2.5, 3);
    cluster_analysis.ExecDBSCANRecursive();
    cluster_analysis.SetClusteredResults(pedestrian_cluested_ids_);

    std::cout << "\n clustering done! clustered pedestrians: " << std::endl;
    for (const auto& item : pedestrian_cluested_ids_) {
        std::cout << " cluster id = " << item.first << ", ped ids:";
        for (const auto& ped : item.second) {
            std::cout << " " << ped;
        }
        std::cout << std::endl;
    }
}

}   // cluster

