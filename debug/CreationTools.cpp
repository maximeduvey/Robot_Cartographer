#include "CreationTools.h"


///
/// generate a simple cube with :
/// points.insert(generateCloudOfPointsForObj( Object3D{pos, size}));
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

    // generate a cross line
    pts = generateLineOfPointsForObj(
        Object3D{
            Eigen::Vector3f(0, 50, 0.0f) + decalage,
            Eigen::Vector3f(25, 1, 1)});
    points.insert(points.end(), pts.begin(), pts.end());
    pts = generateLineOfPointsForObj(
        Object3D{
            Eigen::Vector3f(0, 50, 0.0f) + decalage,
            Eigen::Vector3f(1, 25, 1)});
    points.insert(points.end(), pts.begin(), pts.end());


    // generate a line
        pts = generateLineOfPointsForObj(
        Object3D{
            Eigen::Vector3f(50, 0, 0.0f) + decalage,
            Eigen::Vector3f(25, 1, 1)});
    points.insert(points.end(), pts.begin(), pts.end());

            pts = generateLineOfPointsForObj(
        Object3D{
            Eigen::Vector3f(-50, 0, 0.0f) + decalage,
            Eigen::Vector3f(1, 25, 1)});
    points.insert(points.end(), pts.begin(), pts.end());

    // line in Bias
    pts = generateLineOfPointsFromTo(
            Eigen::Vector3f(-25, -25, 0),
            Eigen::Vector3f(10, -5, 0) );
    points.insert(points.end(), pts.begin(), pts.end());
    

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
                //std::cout << "x:" << point.pos[0] << "y:" << point.pos[1] << "z:" << point.pos[2] << std::endl;
                points.emplace_back(point);
            }
        }
    }

    return points;
}

std::vector<Point> CreationTools::generateLineOfPointsForObj(const Object3D &obj)
{
    std::vector<Point> points;

    // Calculate line start and end based on the object's center and size
    Eigen::Vector3f start = obj.center - obj.size * 0.5f;
    Eigen::Vector3f end = obj.center + obj.size * 0.5f;

    // Number of points along the line
    const float step = 0.5f; // Spacing between points
    float distance = (end - start).norm();
    int numPoints = static_cast<int>(distance / step);

    // Generate points along the line
    Eigen::Vector3f direction = (end - start).normalized();
    for (int i = 0; i <= numPoints; ++i)
    {
        Eigen::Vector3f pos = start + direction * (i * step);
        points.emplace_back(Point{pos, static_cast<uint16_t>(50)});
    }

    return points;
}

std::vector<Point> CreationTools::generateLineOfPointsFromTo(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
{
    std::vector<Point> points;

    // Calculate the direction vector and distance
    Eigen::Vector3f direction = (end - start).normalized();
    float distance = (end - start).norm();

    // Define the step size for point generation
    //const float step = 0.5f; // Spacing between points // working
    const float step = 1; // Spacing between points
    int numPoints = static_cast<int>(distance / step);

    // Generate points along the line
    for (int i = 0; i <= numPoints; ++i)
    {
        Eigen::Vector3f pos = start + direction * (i * step);
        points.emplace_back(Point{pos, static_cast<uint16_t>(50)});
    }

    return points;
}


