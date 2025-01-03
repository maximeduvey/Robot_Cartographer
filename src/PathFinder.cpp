
#include <PathFinder.h>

PathFinder::PathFinder(/* args */)
{
}

PathFinder::~PathFinder()
{
}

// Function to check if the path is clear from the current position to the target
bool PathFinder::isPathClear(
    const Point &target,
    const std::vector<std::vector<int>> &grid,
    const Eigen::Vector2d &currentRobotPosition,
    const Eigen::Vector2d &robotSize, )
{
    // Convert robot size to grid cells
    int safety_radius_x = static_cast<int>(std::ceil(PAMI_ROBOT_SIZE_WIDTH / 2.0 / grid_resolution_));
    int safety_radius_y = static_cast<int>(std::ceil(PAMI_ROBOT_SIZE_LENGTH / 2.0 / grid_resolution_));

    // Convert current robot position to grid coordinates
    int start_x = static_cast<int>(std::round(currentRobotPosition.x() / grid_resolution_)) + (grid_width_ / 2);
    int start_y = static_cast<int>(std::round(currentRobotPosition.y() / grid_resolution_)) + (grid_height_ / 2);

    // Convert target position to grid coordinates
    int target_x = static_cast<int>(std::round(target.x / grid_resolution_)) + (grid_width_ / 2);
    int target_y = static_cast<int>(std::round(target.y / grid_resolution_)) + (grid_height_ / 2);

    // Use Bresenham's line algorithm to trace the path
    int dx = std::abs(target_x - start_x);
    int dy = std::abs(target_y - start_y);
    int sx = (start_x < target_x) ? 1 : -1;
    int sy = (start_y < target_y) ? 1 : -1;
    int err = dx - dy;

    int x = start_x;
    int y = start_y;

    while (true)
    {
        // Check cells within the robot's width and length around the path
        for (int i = -safety_radius_x; i <= safety_radius_x; ++i)
        {
            for (int j = -safety_radius_y; j <= safety_radius_y; ++j)
            {
                int check_x = x + i;
                int check_y = y + j;

                if (check_x >= 0 && check_x < grid_width_ && check_y >= 0 && check_y < grid_height_)
                {
                    if (isgridCellOccupied(check_x, check_y))
                    {
                        // Obstacle found in the safety corridor
                        return false;
                    }
                }
            }
        }

        // Check if the end of the line is reached
        if (x == target_x && y == target_y)
            break;

        // Bresenham’s algorithm step
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    // No obstacles detected in the path
    return true;
}

// Helper function to check if a cell is occupied
bool PathFinder::isgGidCellOccupied(
    const std::vector<std::vector<int>> &grid,
    int grid_x,
    int grid_y)
{
    return grid[grid_y][grid_x] == 1;
}