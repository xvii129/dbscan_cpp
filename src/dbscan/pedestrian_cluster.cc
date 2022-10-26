/******************************************************************************
 * Name: pedestrian cluster analysis source file
 * Copyright: xxx
 * Date: 2022.10.01
 * Description: dense crowd clustering
 *****************************************************************************/

#include "pedestrian_cluster.h"
#include <cmath>

const bool g_pedestrian_cluster_switch = true;

namespace cluster {

int DataPoint::GetDpId() const
{
    return this->target_id_;
}

void DataPoint::SetDpId(int target_id)
{
    this->target_id_ = target_id;
}

const Point2d& DataPoint::GetPosition() const
{
    return this->target_position_;
}

void DataPoint::SetPosition(const Point2d& target_position)
{
    this->target_position_.set_x(target_position.x());
    this->target_position_.set_y(target_position.y());
}

bool DataPoint::IsCorePoint() const
{
    return this->is_core_point_;
}

void DataPoint::SetCorePoint(bool is_core_point)
{
    this->is_core_point_ = is_core_point;
}

bool DataPoint::IsVisited() const
{
    return this->is_visited_;
}

void DataPoint::SetVisited(bool is_visited)
{
    this->is_visited_ = is_visited;
}

int DataPoint::GetClusterId() const
{
    return this->cluster_id_;
}

void DataPoint::SetClusterId(int cluster_id)
{
    this->cluster_id_ = cluster_id;
}

std::vector<int>& DataPoint::GetArrivalPoints()
{
    return this->arrival_points_;
}

void ClusterAnalysis::Init(const std::vector<std::pair<int, Point2d>>& pedestrian_data, double radius, int min_points)
{
    if (pedestrian_data.empty()) {
        return;
    }
    this->epsilon_radius_ = radius;
    this->min_points_ = min_points;
    this->data_num_ = pedestrian_data.size();
    
    for (const auto& pedestrian : pedestrian_data) {
        DataPoint data_point;
        data_point.SetDpId(pedestrian.first);
        data_point.SetClusterId(-1);    // default value
        data_point.SetVisited(false);
        data_point.SetCorePoint(false);
        data_point.SetPosition(pedestrian.second);
        data_set_[pedestrian.first] = data_point;
    }
    for (auto& data_item : data_set_) {
        SetArrivalPoints(data_item.second);
    }
}

double ClusterAnalysis::GetDistance(DataPoint& point1, DataPoint& point2)
{
    double distance = 0.0;
    const auto& position_1 = point1.GetPosition();
    const auto& position_2 = point2.GetPosition();
    return std::hypot(position_2.x() - position_1.x(), position_2.y() - position_1.y());
}

void ClusterAnalysis::SetArrivalPoints(DataPoint& data_point)
{
    for (auto& data_item : data_set_) {
        double distance = GetDistance(data_item.second, data_point);
        if (distance <= epsilon_radius_) {
            // include self-point
            data_point.GetArrivalPoints().emplace_back(data_item.first);
        }
    }
    if (data_point.GetArrivalPoints().size() >= min_points_) {
        data_point.SetCorePoint(true);
        return;
    }
    data_point.SetCorePoint(false);
}

void ClusterAnalysis::KeyPointCluster(DataPoint& data_point, int cluster_id)
{
    if (!data_point.IsCorePoint()) {
        return;
    }
    std::vector<int>& arrival_points = data_point.GetArrivalPoints();
    for (int index = 0; index < arrival_points.size(); ++index) {
        int pedestrian_id = arrival_points[index];
        DataPoint& pedestrian = data_set_[pedestrian_id];
        if (!pedestrian.IsVisited()) {
            pedestrian.SetClusterId(cluster_id);
            pedestrian.SetVisited(true);
            if (pedestrian.IsCorePoint()) {
                // check: in case of stack overflow
                KeyPointCluster(pedestrian, cluster_id);
            }
        }
    }
}

void ClusterAnalysis::ExecDBSCANRecursive()
{
    int cluster_id = 0;
    if (data_num_ != data_set_.size()) {
        return;
    }
    for (auto& data_item : data_set_) {
        DataPoint& data_point = data_item.second;
        if (data_point.IsVisited() || !data_point.IsCorePoint()) {
            continue;
        }
        data_point.SetVisited(true);
        data_point.SetClusterId(cluster_id);
        // clustering recursively
        KeyPointCluster(data_point, cluster_id);
        cluster_id++;
    }
}

void ClusterAnalysis::SetClusteredResults(std::unordered_map<int, std::vector<int>>& pedestrian_cluested_ids)
{
    for (auto& data_item : data_set_) {
        int cluster_id = data_item.second.GetClusterId();
        if (cluster_id == -1) {
            continue;
        }
        pedestrian_cluested_ids[cluster_id].emplace_back(data_item.first);
    }
}

} // cluster