/******************************************************************************
 * Name: pedestrian cluster main function
 * Copyright: xxx
 * Date: 2022.10.01
 * Description: dense crowd clustering
 *****************************************************************************/

#include "obstacles_process.h"
#include <memory>

using cluster::ObstaclesProcessor;

int main()
{
    // 1. 创建obstacle管理实例
    std::unique_ptr<ObstaclesProcessor> obstacles_ptr(nullptr);
    obstacles_ptr.reset(new ObstaclesProcessor);

    // 2. 调用聚类算法实现行人聚类
    obstacles_ptr->Init();
    obstacles_ptr->DenseCrowdAnalysis();

    // 3. 聚类后的行人后处理
    // post process
    return 0;
}

