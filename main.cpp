#include <iostream>
#include <random>
#include "rrt.h"

int main()
{
    constexpr int n = 21;
    std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
    MakeGrid(grid);

    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<int> distr(0, n - 1);

    Node start(distr(eng), distr(eng), 0, 0, 0, 0);
    Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

    start.id_ = start.x_ * n + start.y_;
    start.pid_ = start.x_ * n + start.y_;
    goal.id_ = goal.x_ * n + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;
    PrintGrid(grid);

    std::vector<Node> path_vector;
    std::vector<std::vector<int>> main_grid = grid;

    constexpr double threshold = 2;
    constexpr int max_iter_x_factor = 20;

    grid = main_grid;
    RRT new_rrt;
    path_vector = new_rrt.rrt(grid, start, goal, max_iter_x_factor, threshold);
    PrintPath(path_vector, start, goal, grid);
    return 0;
}
