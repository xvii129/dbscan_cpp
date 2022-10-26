/******************************************************************************
 * Name: obstacles process header file
 * Copyright: xxx
 * Date: 2022.10.01
 * Description: obstacles process
 *****************************************************************************/

#ifndef OBSTACLES_PROCESS_H
#define OBSTACLES_PROCESS_H

#include <memory>
#include <string>
#include <vector>

#include "dbscan/pedestrian_cluster.h"

namespace cluster {

enum Type {
    VEHICLE = 0,
    CYCLIST,
    PEDESTRIAN,
    UNKNOWN,
};

struct Obstacle {
    int id = -1;                // obstacle id
    int type = Type::UNKNOWN;   // obstacle type
    Point2d pose = {0.0, 0.0};  // obstacle position
};

class ObstaclesProcessor {
public:
    /**
     * @brief Constructor
     */
    ObstaclesProcessor();

    /**
     * @brief Destructor
     */
    virtual ~ObstaclesProcessor() = default;

    /**
     * @brief get obstacles
     * @return movable obstacles
     */
    const std::vector<Obstacle>& movable_obstacles() const;

    /**
     * @brief get pedestrian clustered ids
     * @return pedestrian clustered ids
     */
    const std::unordered_map<int, std::vector<int>>& pedestrian_cluested_ids() const;

    /**
     * @brief init obstacles info
     * 
     */
    void Init();

    /**
     * @brief pedestrian cluster analysis
     * 
     */
    void DenseCrowdAnalysis();

private:
    // Obstacle* GetObstacleWithLRUUpdate(const int obstacle_id);

    /**
     * @brief Check if an obstacle is movable
     * @param An obstacle
     * @return True if the obstacle is movable; otherwise false;
     */
    // bool IsMovable(const perception::PerceptionObstacle& perception_obstacle);

private:
    // common::util::LRUCache<int, std::unique_ptr<Obstacle>> ptr_obstacles_;
    // id, type, position
    std::vector<Obstacle> movable_obstacle_ids_;
    std::unordered_map<int, std::vector<int>> pedestrian_cluested_ids_;
    // std::unique_ptr<ObstacleClusters> clusters_;
};

}  // cluster

#endif