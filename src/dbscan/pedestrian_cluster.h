/******************************************************************************
 * Name: pedestrian cluster analysis header file
 * Copyright: xxx
 * Date: 2022.10.01
 * Description: dense crowd clustering
 *****************************************************************************/

#ifndef PEDESTRIAN_CLUSTER_H
#define PEDESTRIAN_CLUSTER_H

#include "math/math.h"

#include <vector>
#include <unordered_map>

extern const bool g_pedestrian_cluster_switch;

/**
 * @namespace cluster
 * @brief cluster
 */
namespace cluster {

using math::Point2d;

class DataPoint {
private:
    int target_id_;                     // pedestrian id
    Point2d target_position_;           // pedestrian position
    int cluster_id_;                    // valid >= 0; -1 for noise point
    bool is_core_point_;                // core point
    bool is_visited_;                   // visited after cluster
    std::vector<int> arrival_points_;   // pedestians within epsilon_radius

public:

    DataPoint() = default;

    ~DataPoint() {
        arrival_points_.clear();
    }

    /**
     * @brief get pedestrian id
     * @return int
     */
    int GetDpId() const;

    /**
     * @brief set pedestrian id
     * @return void
     */
    void SetDpId(int target_id);

    /**
     * @brief get pedestrian position
     * @return Point2d& 
     */
    const Point2d& GetPosition() const;

    /**
     * @brief set pedestrian position
     * @param target_position 
     */
    void SetPosition(const Point2d& target_position);

    /**
     * @brief is core object
     * @return true if core object, otherwise false
     */
    bool IsCorePoint() const;

    /**
     * @brief Set as core point
     * @param is_core_point
     * @return void
     */
     void SetCorePoint(bool is_core_point);
     
    /**
     * @brief is visited when clustering
     * @return true if visited, otherwise false
     */
    bool IsVisited() const;

    /**
     * @brief Set as Visited
     * @param is_visited 
     */
    void SetVisited(bool is_visited);

    /**
     * @brief Get the Cluster Id
     * @return int
     */
    int GetClusterId() const;

    /**
     * @brief Set Cluster Id
     * @param cluster_id 
     */
    void SetClusterId(int cluster_id);

    /**
     * @brief Get the Arrival Points
     * @return vector<int>& 
     */
    std::vector<int>& GetArrivalPoints();
};


class ClusterAnalysis {
private:
    // std::unordered_map<int, DataPoint> data_set_;
    std::unordered_map<int, DataPoint> data_set_;
    double epsilon_radius_;
    int min_points_;
    int data_num_;

    /**
     * @brief calculate distance of two pedestrians
     * @param point1 
     * @param point2 
     * @return double 
     */
    double GetDistance(DataPoint& point1, DataPoint& point2);
    
    /**
     * @brief record pedestrians within epsilon_radius
     * @param data_point pedestrian reference
     * @return void
     */
    void SetArrivalPoints(DataPoint& data_point);

    /**
     * @brief pedestrians clustering within core point epsilon_radius
     * @param data_point pedestrian
     * @param cluster_id
     * @return void
     */
     void KeyPointCluster(DataPoint& data_point, int cluster_id);

public:
 
    ClusterAnalysis() = default;
    
    ~ClusterAnalysis() {
        data_set_.clear();
    }

    /**
     * @brief initialization according to pedestrian data {id, position}
     * @param pedestrian_data pedestrian extracted info
     * @param radius search radius
     * @param min_points min points for core point
     * @return void
     */
    void Init(const std::vector<std::pair<int, Point2d>>& pedestrian_data, double radius = 2.5, int min_points = 3);

    /**
     * @brief execute dbscan algorithm recursively
     * @return void
     */
    void ExecDBSCANRecursive();

    /**
     * @brief record clustered result for post-process
     * @param pedestrian_cluested_ids {cluster_id, vector<pedestrian_id>}
     * @return void
     */
    void SetClusteredResults(std::unordered_map<int, std::vector<int>>& pedestrian_cluested_ids);
};

} // cluster

#endif