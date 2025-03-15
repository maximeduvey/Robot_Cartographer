
#include <gtest/gtest.h>
#include "Mapper.h"
#include "CreationTools.h"


class MapperTest_Friend : public ::testing::Test {
public:
    Mapper _mapper;

    void fillOccupancyGrid(
        std::vector<std::vector<int>>& occupancyGrid,
        const std::vector<pcl::PointIndices> &cluster_indices
    )
    {
        _mapper.fillOccupancyGrid(occupancyGrid, cluster_indices, _mapper._parsingDataPointCloudFiltered);
    }
    
    void getNoiseFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        _mapper.getNoiseFilteredPointCloud(cloud);
    }

    std::vector<std::shared_ptr<Object3D>> refineMapToObjects(const std::vector<std::vector<int>> &grid, size_t lidarCycle, float start_angle, float end_angle)
    {
        return _mapper.refineMapToObjects(grid, lidarCycle, start_angle, end_angle);
    }
    
    void getClusteringFilterEuclidian(std::vector<pcl::PointIndices> &cluster_indices)
    {
        _mapper.getClusteringFilterEuclidian(cluster_indices);
    }

};

TEST_F(MapperTest_Friend, FillOccupancyGrid) {
    std::vector<Point> points;

    Eigen::Vector3f pos(50.0f, 50.0f, 0.0f);
    Eigen::Vector3f size(10.0f, 10.0f, 1.0f);
    // cube
    auto pt = CreationTools::generateCloudOfPointsForObj(Object3D{ pos, size });
    points.insert(points.end(), pt.begin(), pt.end());
    
    FieldPoints fp;
    fp.lidarCycle = 1;
    fp.start_angle = 0.0f;
    fp.end_angle = 15.0f;
    fp.points = points;

    auto parsingDataPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    *parsingDataPointCloud = _mapper.convertToPCLCloud(points);
    getNoiseFilteredPointCloud(parsingDataPointCloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    getClusteringFilterEuclidian(cluster_indices);

    MapSpatialInfos map;
    std::vector<std::vector<int>> occupancyGrid(map.grid_width, std::vector<int>(map.grid_lenght, 0));
    fillOccupancyGrid(occupancyGrid, cluster_indices);

    auto res_pos = pos / map.gridResolution;
    auto res_size = size / map.gridResolution;
    for (size_t width = 0; width < map.grid_width; ++width) 
    {
        for (size_t lenght = 0; lenght < map.grid_lenght; ++lenght) 
        {
            if (occupancyGrid[width][lenght] > 0)
            {
                std::cout << "occupancyGrid["<< width <<"]["<< lenght <<"] = " << occupancyGrid[width][lenght] << std::endl;
                // between
                EXPECT_GE(width, res_pos.x() - res_size.x());
                EXPECT_LE(width, res_pos.x() + res_size.x());
                
                EXPECT_GE(lenght, res_pos.y() - res_size.y());
                EXPECT_LE(lenght, res_pos.y() + res_size.y());
            }
            
        }

    }
    

    auto objs = refineMapToObjects(occupancyGrid, fp.lidarCycle, fp.start_angle, fp.end_angle);

    for (const auto & obj : objs) {
        auto center = obj.get()->center;
        std::cout << "test -- center x:" << center.x() << ", y:" << center.y() << ", z:" << center.z() << ", size:" << obj.get()->size << " -> " << std::endl;
        auto borders = obj.get()->getBoundingBoxCorners();
        int a = 0;
        for (const auto border : borders)
        {
             std::cout << a++  << " - x:" << border.x() << ", y:" << border.y() << ", z:" << border.z() << std::endl;
        }
        EXPECT_FLOAT_EQ((pos.x() + (size.x() / 2)), center.x());
        EXPECT_FLOAT_EQ((pos.y() + (size.y() / 2)), center.y());
        EXPECT_NEAR((pos.z() + (size.z() / 2)), center.z(), 0.5);
    }


    EXPECT_TRUE(true);
}


/** Infos Gtest
 * MapperTest → The test suite name (groups related tests).
 * VectorNormalization → The specific test case name.
 */
TEST_F(MapperTest_Friend, VectorNormalization) {
    Eigen::Vector3f v(3.0f, 4.0f, 0.0f);
    Eigen::Vector3f normalized = v.normalized();
    EXPECT_FLOAT_EQ(normalized.norm(), 1.0f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}