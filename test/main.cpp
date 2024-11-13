
#include "Mapper.h"

#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>

std::vector<Point> generateTestPoints()
{
    std::vector<Point> points;

    // Clustered points around (50, 50)
    for (int i = -3; i <= 3; ++i)
    {
        for (int j = -3; j <= 3; ++j)
        {
            points.push_back(Point{Eigen::Vector2f(50.0f + i * 0.5f, 50.0f + j * 0.5f), static_cast<uint16_t>(50)});
        }
    }

    // Unique outlier points scattered elsewhere
    points.push_back(Point{Eigen::Vector2f(10.0f, 15.0f), 100});
    points.push_back(Point{Eigen::Vector2f(180.0f, 20.0f), 250});
    points.push_back(Point{Eigen::Vector2f(75.0f, 250.0f), 300});
    points.push_back(Point{Eigen::Vector2f(120.0f, 150.0f), 200});
    points.push_back(Point{Eigen::Vector2f(160.0f, 200.0f), 150});

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

    map.addDataToParse(field);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    map.saveOccupancyGridToFile("output.log");
    while (1)
    {
    }

    return 0;
}