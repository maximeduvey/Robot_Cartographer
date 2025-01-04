#pragma once

#include <CommonSpaceRepresentation.h>
#include <MapSpatialInfos.h>
#include <RobotSpatialInfos.h>

#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/visualization/pcl_visualizer.h>

#define MAPPER_MAX_FIELD_VIEW 360.0f
#define MAPPER_MIN_FIELD_VIEW 0.0f

#define MAPPER_GRID_RESOLUTION 1.0f // each grid square will be of 1cm square
// define how many number of point need to be in a section to be considered a "cluster" and translated to an object
// it's managed my pcl with setMinClusterSize, and can depend on how we proceed the lidar data
// currently set as one, but, to my point of view, it should alwais be highier than one (to prevent some risk of false data poluting)
// but in the meanwhile, set to one for classic lidar data (not high resolution)
#define MAPPER_GRID_RESOLUTION_CLUSTER_POINT 1.0f
#define MAPPER_GRID_RESOLUTION_CLUSTER_DISTANCE_TOLERANCE 1.0f
// a cluster with to many point is skeep, this is the threshold, for lidar data it's way beyond what is needed
#define MAPPER_GRID_RESOLUTION_CLUSTER_NUMBER_TOLERANCE 25000

#define MAPPER_CLOUD_POINT_FILTERING_THRESHOLD 5
#define MAPPER_DEFAULT_WAITING_TIME_PARSER_EMPTY_LIST 1000 // 1sec

// This is the threshold for matching 2 consecutive detected objects (tracking of object)
#define MAPPER_MARGING_DETECTION_THRESHOLD_MERGER 1.0f
#define MAPPER_MAX_DEPTH_PATH_NUMBER_POINT 50

// this is the distance (from an corner pos) we consider to be "as if" we are on this corner 
#define MAPPER_THRESHOLD_DISTANCE_FOR_CORNER 1.0f

static int _detected_object_id_incrementer = 0;
inline int getObjectNewId()
{
    ++_detected_object_id_incrementer;
    if (_detected_object_id_incrementer >= INT_MAX)
        _detected_object_id_incrementer = 0;
    return _detected_object_id_incrementer;
}

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
    using FieldPointsCallback = std::function<void(const FieldPoints&)>;

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

    bool lineIntersectsAABB(const Object3D &movingObj,
                            const Object3D &immobileObj,
                            const Eigen::Vector3f &destination);

    /// Inliner ///
    inline bool isRunning() { return _end.load(); }
    ///         ///

    // Function to set the callback
    inline void setFieldPointsCallback(FieldPointsCallback callback) { _fieldPointsCallback_dataProcessed = callback; };
private:
    static void loop_parseFieldPoints(Mapper *myself);

    void parsePointToMap_pointsAreRelativeToRobot(const Point &point);
    void parsePointToMap_pointsAreAbsolute(const Point &point);

    pcl::PointCloud<pcl::PointXYZ> convertToPCLCloud(const std::vector<Point> &lidarPoints);
    bool checkCollision(const Eigen::Vector2f &robotPos, const Eigen::Vector3f &robotSize, const Object3D &object);

    static Eigen::Vector3f getCollidingNextPositionCloserBorderLogic(const RobotSpatialInfos &robot,
                                                              const Object3D &collidingObject,
                                                              const Eigen::Vector3f &destination);

    void processLidarData(const std::vector<Point> &lidarPoints);
    Point transformPointToGlobal(const Point &point);
    void updateOccupancyGrid(const Point &global_point);

    bool isInCollision(const Eigen::Vector3f &position, const Eigen::Vector3f &size, const Eigen::Vector3f &point);
    void recursiveCalculateNextPathPositionToGoal(const RobotSpatialInfos &robot,
                                                  const Eigen::Vector3f &destination,
                                                  const std::vector<Object3D> &objects,
                                                  std::vector<Eigen::Vector3f> &pathToFill);

    std::vector<std::pair<int, int>> findPath(
        const std::vector<std::vector<int>> &grid,
        int startX, int startY, int destX, int destY);

    void linkDetectedObjects(std::vector<Object3D> &refined_currentDetectedObject,
                             const std::vector<Object3D> &refined_lastDetectedObject,
                             const Eigen::Vector3f &robotMovement);

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

private:
    std::mutex _mutexDetectedObject;
    std::vector<Object3D> refined_currentDetectedObject;
    std::vector<Object3D> refined_lastDetectedObject;

    // callback to be informed when the data from the Lidar has been proccessed, will surely be replaced
    FieldPointsCallback _fieldPointsCallback_dataProcessed;

    // a list of points to reach the current goal destination
    std::mutex _mutexPointsPathToDest;
    std::vector<Eigen::Vector3f> _pointsPathToDest;

    /// this allow us to shift the detected points to the "center" of our physical c++ map
    /// our perception is around the robot, so points can be negative but our map start at 0
    Eigen::Vector3f _mapCenterShifter = {0, 0, 0};
};
