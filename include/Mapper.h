#pragma once

#include "CommonSpaceRepresentation.h"
#include "MapSpatialInfos.h"

#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#define MAPPER_MAX_FIELD_VIEW 360.0f
#define MAPPER_MIN_FIELD_VIEW 0.0f

#define MAPPER_GRID_RESOLUTION 1.0f // each grid square will be of 1cm square

#define MAPPER_CLOUD_POINT_FILTERING_THRESHOLD 5

#define MAPPER_DEFAULT_WAITING_TIME_PARSER_EMPTY_LIST 1000 // 1sec

/// @brief this class manage the map of object that is perceived around the robot
/// in short it is feed with field of points (that each have a x, Y position)
/// it will then update an accessible map of occupied block of space.
///
/// For now it does not manage orientation as we do not have a correct way of perceiving it
/// But keep in mind that, as it is, the map is more a cone of view, than an coherent register of perceived object
///
class Mapper
{
private:
    struct Node
    {
        int x, y;
        float cost, heuristic;
        Node *parent;

        Node(int x, int y, float cost, float heuristic, Node *parent = nullptr)
            : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

        float totalCost() const { return cost + heuristic; }
    };

    // Comparison operator for priority queue (min-heap)
    struct CompareNode
    {
        bool operator()(const Node *a, const Node *b) const
        {
            return a->totalCost() > b->totalCost();
        }
    };

public:
    Mapper(MapSpatialInfos map = MapSpatialInfos(),
           float startAngleManaged = MAPPER_MIN_FIELD_VIEW,
           float endAngleManaged = MAPPER_MAX_FIELD_VIEW);
    virtual ~Mapper();

    void setManagedAngleFieldOFView(float startAngleManaged, float endAngleManaged);
    void setMapInfos(MapSpatialInfos map = MapSpatialInfos());

    void enableNoiseFilter(bool enable = true);
    void setNoiseFilterNbrCorelationPoint(int nbr);

    void startDataParsing();
    void stopDataParsing();

    bool isCellBlockedWithRobotSize(int x, int y,
                                    const std::vector<std::vector<int>> &grid);

    void addDataToParse(const FieldPoints &fieldPoints);
    std::vector<Object3D> refineMapToObjects(const std::vector<std::vector<int>> &grid);

    void saveOccupancyGridToFile(const std::string &filename);

    /// Inliner ///
    inline bool isRunning() { return _end.load(); }
    ///         ///
private:
    static void loop_parseFieldPoints(Mapper *myself);

    void parsePointToMap_pointsAreRelativeToRobot(const Point &point);
    void parsePointToMap_pointsAreAbsolute(const Point &point);

    pcl::PointCloud<pcl::PointXYZ> convertToPCLCloud(const std::vector<Point> &lidarPoints);
    bool checkCollision(const Eigen::Vector2f &robotPos, const Eigen::Vector3f &robotSize, const Object3D &object);
    std::vector<Eigen::Vector2f> findPathWithVectorCalculation(
        const Eigen::Vector2f &start,
        const std::vector<Object3D> &obstacles);

    void processLidarData(const std::vector<Point> &lidarPoints);
    Point transformPointToGlobal(const Point &point);
    void updateOccupancyGrid(const Point &global_point);

    float getHeuristic(const Eigen::Vector2f &current, const Eigen::Vector2f &destination);

    // void updateGridWithClusters(const std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    std::vector<std::pair<int, int>> findPath(
        const std::vector<std::vector<int>> &grid,
        int startX, int startY, int destX, int destY);

public:
    float _startAngleManaged;
    float _endAngleManaged;

    /// TODO: the following info should centralized in PAMI and shared accross with pointer or a singleton (i'm more for singleton)
    // is bound to stay at 0 (as explained in the class description) but i still want it
    // std::atomic<float> _currentRobotAngle = {0.0f};
    // // This the current robot position, it's also
    // Eigen::Vector2d _currentRobotPosition = {0.0f, 0.0f};
    // // this is the robots size, it is used to determine a collision path to the objective
    // Eigen::Vector2d _robotSize = {PAMI_ROBOT_SIZE_LENGTH, PAMI_ROBOT_SIZE_WIDTH};
    ///
    RobotSpatialInfos _robotInfos; // this need to be centralized in PAMI

    /// @brief this the vector that will be continuously parsed to update the map
    std::vector<FieldPoints> _dataToParse;
    // const pcl::PointCloud<pcl::PointXYZ>::Ptr& _cloudPoint;
    std::mutex _mutextDataToParse;

    std::atomic<bool> _end;
    std::thread _parserData;

    std::atomic<bool> _noiseFilter_Enable = {true};
    std::atomic<int> _noiseFilter_nbrCorelationPoint = {MAPPER_CLOUD_POINT_FILTERING_THRESHOLD};

    /// GRID ///
    // float _grid_resolution = MAPPER_GRID_RESOLUTION;
    // int _grid_width = MAP_WIDTH;
    // int _grid_lenght = MAP_LENGHT;
    MapSpatialInfos _map;
    std::vector<std::vector<std::pair<bool, int>>> _occupancy_grid;
    ///     ///
};
