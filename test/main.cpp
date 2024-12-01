
#include "Mapper.h"

#include "CommonDebugFunction.h"

#include "CreationTools.h"

int main()
{
    std::cout << TAG << std::endl;
    auto vec = CreationTools::generateTestPoints();

    Mapper map;
    map.startDataParsing();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    FieldPoints field;
    field.points = vec;

    CommonDebugFunction::savePointsToFile(vec, "vec.log");

    map.addDataToParse(field);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    //CommonDebugFunction::saveOccupancyGridToFile(map._occupancy_grid, "output.log", {0,0,0},{0,0,0});
    while (1)
    {
    }

    return 0;
}

