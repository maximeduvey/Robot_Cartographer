
#include "Mapper.h"

#include "CommonDebugFunction.h"

std::vector<Point> generateTestPoints()
{
    std::vector<Point> points;


    // Clustered points around (50, 50)
    for (int i = -10; i <= 10; ++i)
    {
        for (int j = -10; j <= 10; ++j)
        {
            points.push_back(Point{Eigen::Vector2f(50.0f + i * 0.5f, 50.0f + j * 0.5f), static_cast<uint16_t>(50)});
        }
    }

    for (int i = -10; i <= 10; ++i)
    {
        for (int j = -10; j <= 10; ++j)
        {
            points.push_back(Point{Eigen::Vector2f(80.0f + i * 0.5f, 80.0f + j * 0.5f), static_cast<uint16_t>(50)});
        }
    }

    // Unique outlier points scattered elsewhere
    points.push_back(Point{Eigen::Vector2f(10.0f, 15.0f), 100});
    points.push_back(Point{Eigen::Vector2f(180.0f, 20.0f), 250});
    points.push_back(Point{Eigen::Vector2f(75.0f, 250.0f), 300});
    points.push_back(Point{Eigen::Vector2f(120.0f, 150.0f), 200});
    points.push_back(Point{Eigen::Vector2f(160.0f, 200.0f), 150});

    
    points.push_back(Point{Eigen::Vector2f(1.0f, 1.0f), 100});
    points.push_back(Point{Eigen::Vector2f(2.0f, 2.0f), 250});
    points.push_back(Point{Eigen::Vector2f(3.0f, 3.0f), 300});
    points.push_back(Point{Eigen::Vector2f(4.0f, 4.0f), 200});
    points.push_back(Point{Eigen::Vector2f(5.0f, 5.0f), 150});

    return points;
}

int main()
{
    std::cout << TAG << std::endl;
    auto vec = generateTestPoints();

    Mapper map;
    map.startDataParsing();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    FieldPoints field;
    field.points = vec;

    CommonDebugFunction::savePointsToFile(vec, "vec.log");

    map.addDataToParse(field);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    CommonDebugFunction::saveOccupancyGridToFile(map._occupancy_grid, "output.log");
    while (1)
    {
    }

    return 0;
}

