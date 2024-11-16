
#include "Mapper.h"

#include <fstream>
#include <vector>
#include <iostream>
#include <queue>

//todelete
#include "CommonDebugFunction.h"

Mapper::Mapper(MapSpatialInfos map /* = MapSpatialInfos() */,
               float startAngleManaged /*= MAPPER_MIN_FIELD_VIEW*/,
               float endFieldOfView /* = MAPPER_MAX_FIELD_VIEW*/)
{
    std::cout << TAG << std::endl;
    setManagedAngleFieldOFView(startAngleManaged, endFieldOfView);
    _end.store(false);
    _occupancy_grid = std::vector<std::vector<std::pair<bool, int>>>(
        map.grid_lenght,
        std::vector<std::pair<bool, int>>(map.grid_width, {false, 0}));
    
}

Mapper::~Mapper()
{
}

void Mapper::setManagedAngleFieldOFView(float startAngleManaged, float endAngleManaged)
{
    std::cout << TAG << std::endl;
    _startAngleManaged = startAngleManaged;
    _endAngleManaged = endAngleManaged;
}

void Mapper::setMapInfos(MapSpatialInfos map /*  = MapSpatialInfos() */)
{
    std::cout << TAG << std::endl;
    _map = map;
}

void Mapper::startDataParsing()
{
    std::cout << TAG << std::endl;
    _end.store(false);
    _parserData = std::thread(&Mapper::loop_parseFieldPoints, this);
    _parserData.detach();
}

void Mapper::stopDataParsing()
{
    _end.store(true);
}

/// @brief Once enable, during parsing process, alone point will be filtered and skipped as artefact of the captor
///         point are considered "trusworthy" when there is at least _noiseFilter_nbrCorelationPoint point around them
/// @param enable
void Mapper::enableNoiseFilter(bool enable /*  = true */)
{
    _noiseFilter_Enable.store(enable);
}

/// @brief this allow you to specify the number of point needed around another point
///         fo it to be considered, and not discarded as Noise/artificat of the captor
void Mapper::setNoiseFilterNbrCorelationPoint(int nbr)
{
    _noiseFilter_nbrCorelationPoint.store(nbr);
}

// for debug



void Mapper::loop_parseFieldPoints(Mapper *myself)
{
    
    std::cout << TAG << std::endl;
    while (myself->_end.load() != true)
    {
        myself->_mutextDataToParse.lock();
        auto nbr = myself->_dataToParse.size();
        myself->_mutextDataToParse.unlock();
        if (nbr > 0)
        {
            myself->_mutextDataToParse.lock();
            std::cout << TAG << "_dataToParse contain:" << myself->_dataToParse.size() << std::endl;
            for (const auto &fieldpoints : myself->_dataToParse)
            {
                /* for (const auto &point : fieldpoints.points) {
                    myself->parsePointToMap_pointsAreRelativeToRobot(point);
                } */
                myself->processLidarData(fieldpoints.points);
            }
            myself->_dataToParse.clear();
            std::cout << TAG << "after" << myself->_dataToParse.size() << std::endl;
            myself->_mutextDataToParse.unlock();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(MAPPER_DEFAULT_WAITING_TIME_PARSER_EMPTY_LIST));
        }
    }
}

void Mapper::addDataToParse(const FieldPoints &fieldPoints)
{
    std::cout << TAG << std::endl;
    _mutextDataToParse.lock();
    _dataToParse.push_back(fieldPoints);
    _mutextDataToParse.unlock();
}

/// @brief Points are relative to the robot and are relative to it's rotation
/// @param point
void Mapper::parsePointToMap_pointsAreRelativeToRobot(const Point &point)
{
    //std::cout << TAG << std::endl;
    const auto absPoint = transformPointToGlobal(point);
    updateOccupancyGrid(absPoint);
}

/// @brief Points x,y are aboslute and does not depend of the robot position or rotation
/// @param point
void Mapper::parsePointToMap_pointsAreAbsolute(const Point &point)
{
}

// @brief used to convert a relative-to-robot point to an absolute one
// @param point
Point Mapper::transformPointToGlobal(const Point &point)
{
    Point ret = point;
    // Convert robot-relative point to a global map point
    float theta = _robotInfos._currentRobotAngle;
    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);

    ret.pos[POS_X] = point.pos[POS_X] * cos_theta - point.pos[POS_X] * sin_theta + _robotInfos.currentRobotPosition[POS_X];  
    ret.pos[POS_Y] = point.pos[POS_X] * sin_theta + point.pos[POS_Y] * cos_theta + _robotInfos.currentRobotPosition[POS_Y];

    return ret;
}

void Mapper::updateOccupancyGrid(const Point &global_point)
{
    // Convert global point coordinates to grid cell indices
    int grid_x = static_cast<int>(std::round(global_point.pos[POS_X] / _map.gridResolution)) + (_map.grid_width / 2);
    int grid_y = static_cast<int>(std::round(global_point.pos[POS_Y] / _map.gridResolution)) + (_map.grid_lenght / 2);
    if (grid_x >= 0 && grid_x < _map.grid_width  && grid_y >= 0 && grid_y < _map.grid_lenght)
    {
        _occupancy_grid[grid_y][grid_x].second += 1;
    }
}

pcl::PointCloud<pcl::PointXYZ> Mapper::convertToPCLCloud(const std::vector<Point> &lidarPoints)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>();
    for (const auto &pt : lidarPoints)
    {
        cloud.points.emplace_back(pt.pos[POS_X], pt.pos[POS_Y], 0.0f); // z=0 for 2D data
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    return cloud;
}



/// @brief this function will basically noise filter the data and fill occupancyGrid
/// @param lidarPoints
void Mapper::processLidarData(const std::vector<Point> &lidarPoints)
{
    // Convert lidar data to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud = convertToPCLCloud(lidarPoints); // Assuming convertToPCLCloud correctly converts the vector

    CommonDebugFunction::savePointCloudToFile(*cloud, "cloud.log");

    // 1. Noise Filtering using Statistical Outlier Removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(_noiseFilter_nbrCorelationPoint.load()); // Set number of neighbors to analyze
    sor.setStddevMulThresh(1.0);                          // Threshold for outliers
    sor.filter(*cloud_filtered);
    
    CommonDebugFunction::savePointCloudToFile(*cloud_filtered, "cloud_filtered.log");

    // 2. Clustering using Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(1); // Distance tolerance for clustering
    ec.setMinClusterSize(5);     // Minimum number of points for a valid cluster
    ec.setMaxClusterSize(500);   // Maximum number of points per cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    CommonDebugFunction::savePointCloudToFile(*cloud_filtered, "cloud_filtered2.log");

    // 3. Create an Occupancy Grid
    std::cout << "cluster_indices.size : " << cluster_indices.size() << std::endl;
    std::vector<std::vector<int>> occupancyGrid(_map.grid_width, std::vector<int>(_map.grid_lenght, 0));
    for (const auto &cluster : cluster_indices) {
        for (const int &index : cluster.indices) {
            pcl::PointXYZ point = cloud_filtered->points[index];


            // Convert point coordinates to grid indices
            int gridX = static_cast<int>(point.x / _map.gridResolution);
            int gridY = static_cast<int>(point.y / _map.gridResolution);

            std::cout << "point x:" << point.x << ", point.y:" << point.y << ", gridX:" << gridX << ", gridy: " << gridY << std::endl;

            // Check bounds before writing to the grid
            if (gridX >= 0 && gridX < _map.grid_width && gridY >= 0 && gridY < _map.grid_lenght) {
                occupancyGrid[gridX][gridY] += 1;
            }
        }
    }

    // Optional: Save the occupancy grid for debugging
    CommonDebugFunction::saveOccupancyGridToFile(occupancyGrid, "occupancy_grid.log");

    // // Run Pathfinding with Occupancy Grid
    auto path = findPath(occupancyGrid, 0, 50, 100, 50);
    CommonDebugFunction::savePathToPointCloud(path, "path.log");

    CommonDebugFunction::visualizeOccupancyGridAndPath(occupancyGrid, path, "objectAndPath");

    // updateGridWithClusters(cluster_indices, cloud_filtered);
}

std::vector<std::pair<int, int>> Mapper::findPath(
    const std::vector<std::vector<int>>& grid,
    int startX, int startY, int destX, int destY)
{
    int rows = grid.size();
    int cols = grid[0].size();
    auto heuristic = [](int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);
    };
    // Priority queue for open list
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
    std::unordered_map<int, Node*> allNodes;
    auto hash = [cols](int x, int y) { return x * cols + y; };
    // Start and destination nodes
    Node* startNode = new Node(startX, startY, 0, heuristic(startX, startY, destX, destY));
    openList.push(startNode);
    allNodes[hash(startX, startY)] = startNode;
    std::vector<std::pair<int, int>> directions{{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();
        // Check if we reached the destination
        if (current->x == destX && current->y == destY) {
            std::vector<std::pair<int, int>> path;
            for (Node* node = current; node; node = node->parent) {
                path.emplace_back(node->x, node->y);
            }
            std::reverse(path.begin(), path.end());
            // Cleanup all nodes
            for (auto& [_, node] : allNodes) delete node;
            return path;
        }
        // Explore neighbors
        for (const auto& [dx, dy] : directions) {
            int nx = current->x + dx, ny = current->y + dy;
            // Boundary and obstacle check
            if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && grid[nx][ny] == 0) {
                float newCost = current->cost + 1.0f;
                int nodeHash = hash(nx, ny);
                if (allNodes.find(nodeHash) == allNodes.end() || newCost < allNodes[nodeHash]->cost) {
                    Node* neighbor = new Node(nx, ny, newCost, heuristic(nx, ny, destX, destY), current);
                    openList.push(neighbor);
                    allNodes[nodeHash] = neighbor;
                }
            }
        }
    }
    // Cleanup all nodes if no path is found
    for (auto& [_, node] : allNodes) delete node;
    return {}; // Return empty path if no route to destination
}

// // do not use
// void Mapper::updateGridWithClusters(const std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//     // Loop over each cluster of points
//     for (const auto &cluster : clusters)
//     {
//         // Calculate the bounding box of each cluster
//         float min_x = std::numeric_limits<float>::max();
//         float min_y = std::numeric_limits<float>::max();
//         float max_x = std::numeric_limits<float>::lowest();
//         float max_y = std::numeric_limits<float>::lowest();

//         for (const auto &idx : cluster.indices)
//         {
//             auto &pt = cloud->points[idx];
//             min_x = std::min(min_x, pt.x);
//             min_y = std::min(min_y, pt.y);
//             max_x = std::max(max_x, pt.x);
//             max_y = std::max(max_y, pt.y);
//         }

//         // Mark the occupancy grid within the bounding box as occupied
//         int grid_min_x = static_cast<int>(std::floor(min_x / gridResolution_)) + gridWidth_ / 2;
//         int grid_min_y = static_cast<int>(std::floor(min_y / gridResolution_)) + gridHeight_ / 2;
//         int grid_max_x = static_cast<int>(std::ceil(max_x / gridResolution_)) + gridWidth_ / 2;
//         int grid_max_y = static_cast<int>(std::ceil(max_y / gridResolution_)) + gridHeight_ / 2;

//         for (int i = grid_min_x; i <= grid_max_x; ++i)
//         {
//             for (int j = grid_min_y; j <= grid_max_y; ++j)
//             {
//                 if (i >= 0 && i < gridWidth_ && j >= 0 && j < gridHeight_)
//                 {
//                     // Mark cell as occupied (this example simply prints the coordinates)
//                     std::cout << "Marking grid cell (" << i << ", " << j << ") as occupied\n";
//                 }
//             }
//         }
//     }
// }