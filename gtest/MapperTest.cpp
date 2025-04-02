
#include <gtest/gtest.h>
#include "Mapper.h"
#include "CreationTools.h"

#define DEBUG_LOG true
#define SHOW_DEBUG_LOG if(DEBUG_LOG==true)std::cout 

class MapperTest_Friend : public ::testing::Test {
public:
    Mapper _mapper;

    void fillOccupancyGrid(
        std::vector<std::vector<int>>& occupancyGrid,
        const std::vector<pcl::PointIndices>& cluster_indices
    )
    {
        _mapper.fillOccupancyGrid(occupancyGrid, cluster_indices, _mapper._parsingDataPointCloudFiltered);
    }

    void getNoiseFilteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        _mapper.getNoiseFilteredPointCloud(cloud);
    }

    std::vector<std::shared_ptr<Object3D>> refineMapToObjects(const std::vector<std::vector<int>>& grid, size_t lidarCycle, float start_angle, float end_angle)
    {
        return _mapper.refineMapToObjects(grid, lidarCycle, start_angle, end_angle);
    }

    void getClusteringFilterEuclidian(std::vector<pcl::PointIndices>& cluster_indices)
    {
        _mapper.getClusteringFilterEuclidian(cluster_indices);
    }


    std::vector<std::shared_ptr<Object3D>> createGridAndRefineObject(MapSpatialInfos& map,  Eigen::Vector3f pos,  Eigen::Vector3f size)
    {
        std::vector<Point> points;
        _mapper.setMapInfos(map);
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
                    SHOW_DEBUG_LOG << "occupancyGrid[" << width << "][" << lenght << "] = " << occupancyGrid[width][lenght] << std::endl;
                    // between
                    EXPECT_GE(width, res_pos.x() - res_size.x() + _mapper._mapCenterShifter.x() );
                    EXPECT_LE(width, res_pos.x() + res_size.x() + _mapper._mapCenterShifter.x());
                    EXPECT_GE(lenght, res_pos.y() - res_size.y() + _mapper._mapCenterShifter.y());
                    EXPECT_LE(lenght, res_pos.y() + res_size.y() + _mapper._mapCenterShifter.y());
                }
            }
        }
        auto objs = refineMapToObjects(occupancyGrid, fp.lidarCycle, fp.start_angle, fp.end_angle);
        return objs;
    }

};


TEST_F(MapperTest_Friend, FillOccupancyGrid_with_resolution_of_ONE)
{
    MapSpatialInfos map;
    map.gridResolution = 1.0f;

    Eigen::Vector3f pos(50.0f, 50.0f, 0.0f);
    Eigen::Vector3f size(10.0f, 10.0f, 1.0f);

    auto objs = createGridAndRefineObject(map, pos, size);

    for (const auto& obj : objs) {
        auto center = obj.get()->center;
        SHOW_DEBUG_LOG << "test -- center x:" << center.x() << ", y:" << center.y() << ", z:" << center.z() << ", size:" << obj.get()->size << " -> " << std::endl;
        auto borders = obj.get()->getBoundingBoxCorners();
        EXPECT_EQ(borders.size(), 8);
        int a = 0;
        for (const auto& border : borders)
        {
            SHOW_DEBUG_LOG << a++ << " - x:" << border.x() << ", y:" << border.y() << ", z:" << border.z() << std::endl;
        }
    }
    auto borders = objs[0].get()->getBoundingBoxCorners();
    auto center = borders[0];
    EXPECT_FLOAT_EQ(center.x(), 45.0f);
    EXPECT_FLOAT_EQ(center.y(), 45.0f);
    EXPECT_FLOAT_EQ(center.z(), -0.5f);

    center = borders[6];
    EXPECT_FLOAT_EQ(center.x(), 56.0);
    EXPECT_FLOAT_EQ(center.y(), 56.0);
    EXPECT_FLOAT_EQ(center.z(), 0.5);
}

TEST_F(MapperTest_Friend, FillOccupancyGrid_with_resolution_of_TEN)
{
    std::vector<Point> points;

    Eigen::Vector3f pos(50.0f, 50.0f, 0.0f);
    Eigen::Vector3f size(10.0f, 10.0f, 1.0f);
    
    MapSpatialInfos map;
    map.gridResolution = 10.0f;

    auto objs = createGridAndRefineObject(map, pos, size);

    for (const auto & obj : objs) {
        auto center = obj.get()->center;
        SHOW_DEBUG_LOG << "test -- center x:" << center.x() << ", y:" << center.y() << ", z:" << center.z() << ", size:" << obj.get()->size << " -> " << std::endl;
        auto borders = obj.get()->getBoundingBoxCorners();
        EXPECT_EQ(borders.size(), 8);
        int a = 0;
        for (const auto &border : borders)
        {
            SHOW_DEBUG_LOG << a++  << " - x:" << border.x() << ", y:" << border.y() << ", z:" << border.z() << std::endl;
        }
/*         EXPECT_NEAR((pos.x() + (size.x() / 2)), center.x(), ((size.x() * 0.5f ) / map.gridResolution) + 0.01f);
        EXPECT_NEAR((pos.y() + (size.y() / 2)), center.y(), ((size.y() * 0.5f ) / map.gridResolution) + 0.01f);
        EXPECT_NEAR((pos.z() + (size.z() / 2)), center.z(), ((size.z() * 0.5f ) / map.gridResolution) + 0.01f); // map.gridResolution */
    }
    auto borders = objs[0].get()->getBoundingBoxCorners();
    auto center = borders[0];
    EXPECT_FLOAT_EQ(center.x(), 40.0f);
    EXPECT_FLOAT_EQ(center.y(), 40.0f);
    EXPECT_FLOAT_EQ(center.z(), -5);

    center = borders[6];
    EXPECT_FLOAT_EQ(center.x(), 60.0);
    EXPECT_FLOAT_EQ(center.y(), 60.0);
    EXPECT_FLOAT_EQ(center.z(), 5);
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