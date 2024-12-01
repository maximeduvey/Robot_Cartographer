#include "CreationTools.h"

std::vector<Point> CreationTools::generateTestPoints(Eigen::Vector3f decalage /* = {0.0f, 0.0f, 0.0f} */)
{
    std::vector<Point> points;
    // Clustered points around (50, 50)

    // generate a cube
    int cubePos = 50, cubeSize = 10;
    points = generateCloudOfPointsForObj(
        Object3D{
            Eigen::Vector3f(cubePos + cubePos / 2.0f, cubePos + cubePos / 2, 0.0f) + decalage,
            Eigen::Vector3f(cubeSize, cubeSize, cubeSize)});

    cubePos = 80;
    auto pts = generateCloudOfPointsForObj(
        Object3D{
            Eigen::Vector3f(cubePos + cubePos / 2, cubePos + cubePos / 2, 0.0f) + decalage,
            Eigen::Vector3f(cubeSize, cubeSize, cubeSize)});
    points.insert(points.end(), pts.begin(), pts.end());

/*   cubePos = 50;
    pts = generateCloudOfPointsForObj(
        Object3D{
            Eigen::Vector3f(cubePos, cubePos, 0.0f) + decalage,
            Eigen::Vector3f(cubeSize, cubeSize, cubeSize)});
    points.insert(points.end(), pts.begin(), pts.end());
 */
    for (int i = 0; i < 10; ++i)
    {
        std::cout << points[i].pos << "->" << points[i].distance << std::endl;
    }
    // points.clear();
    std::cout << "-------------" << std::endl;
    for (int i = 0; i < 10; ++i)
    {
        std::cout << points[i].pos << "->" << points[i].distance << std::endl;
    }

    // Unique outlier points scattered elsewhere
    points.push_back(Point{Eigen::Vector3f(10.0f, 15.0f, 0.0f), 100});
    points.push_back(Point{Eigen::Vector3f(180.0f, 20.0f, 0.0f), 250});
    points.push_back(Point{Eigen::Vector3f(75.0f, 250.0f, 0.0f), 300});
    points.push_back(Point{Eigen::Vector3f(120.0f, 150.0f, 0.0f), 200});
    points.push_back(Point{Eigen::Vector3f(160.0f, 200.0f, 0.0f), 150});
    points.push_back(Point{Eigen::Vector3f(1.0f, 1.0f, 0.0f), 100});
    points.push_back(Point{Eigen::Vector3f(2.0f, 2.0f, 0.0f), 250});
    points.push_back(Point{Eigen::Vector3f(3.0f, 3.0f, 0.0f), 300});
    points.push_back(Point{Eigen::Vector3f(4.0f, 4.0f, 0.0f), 200});
    points.push_back(Point{Eigen::Vector3f(5.0f, 5.0f, 0.0f), 150});

    return points;
}

/// @brief create a cloud of point representing the desired object
/// @param obj
/// @return
std::vector<Point> CreationTools::generateCloudOfPointsForObj(const Object3D &obj)
{
    std::vector<Point> points;

    // Calculate boundaries
    Eigen::Vector3i minCorner = (obj.center - obj.size * 0.5f).array().floor().cast<int>();
    Eigen::Vector3i maxCorner = (obj.center + obj.size * 0.5f).array().ceil().cast<int>();

    // Iterate over the bounding box and generate points

    for (float x = minCorner.x(); x <= maxCorner.x(); x += 0.5f)
    {
        for (float y = minCorner.y(); y <= maxCorner.y(); y += 0.5f)
        {
            for (float z = minCorner.z(); z <= maxCorner.z(); z += 0.5f)
            {
                auto point = Point{Eigen::Vector3f(x, y, z), static_cast<uint16_t>(50)};
                std::cout << "x:" << point.pos[0] << "y:" << point.pos[1] << "z:" << point.pos[2] << std::endl;
                points.emplace_back(point);
            }
        }
    }

    return points;
}
