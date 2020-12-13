#include <iomanip> 
#include <iostream>
#include <random>
#include "utils.h"

constexpr int spacing_for_grid = 10;

Node::Node(const int x, const int y, const double cost, const double h_cost,const int id, const int pid) 
{
    this->x_ = x;
    this->y_ = y;
    this->cost_ = cost;
    this->h_cost_ = h_cost;
    this->id_ = id;
    this->pid_ = pid;
}

void Node::PrintStatus() const 
{
    std::cout << "--------------" << std::endl
        << "Node          :" << std::endl
        << "x             : " << x_ << std::endl
        << "y             : " << y_ << std::endl
        << "Cost          : " << cost_ << std::endl
        << "Heuristic cost: " << h_cost_ << std::endl
        << "Id            : " << id_ << std::endl
        << "Pid           : " << pid_ << std::endl
        << "--------------" << std::endl;
}

Node Node::operator+(const Node& p) const 
{
    Node tmp;
    tmp.x_ = this->x_ + p.x_;
    tmp.y_ = this->y_ + p.y_;
    tmp.cost_ = this->cost_ + p.cost_;
    return tmp;
}

Node Node::operator-(const Node& p) const 
{
    Node tmp;
    tmp.x_ = this->x_ - p.x_;
    tmp.y_ = this->y_ - p.y_;
    return tmp;
}

bool compareCoordinates(const Node& p1, const Node& p2)
{
    return p1.x_ == p2.x_ && p1.y_ == p2.y_;
}
bool checkOutsideBoundary(const Node& node, const int n)
{
    return (node.x_ < 0 || node.y_ < 0
        || node.x_ >= n || node.y_ >= n);
}


bool compare_cost::operator()(const Node& p1, const Node& p2) const 
{
    return p1.cost_ + p1.h_cost_ > p2.cost_ + p2.h_cost_ ||
        (p1.cost_ + p1.h_cost_ == p2.cost_ + p2.h_cost_ &&
            p1.h_cost_ >= p2.h_cost_);
}

std::vector<Node> GetMotion() 
{
    Node down(0, 1, 1, 0, 0, 0);
    Node up(0, -1, 1, 0, 0, 0);
    Node left(-1, 0, 1, 0, 0, 0);
    Node right(1, 0, 1, 0, 0, 0);
    std::vector<Node> v;
    v.push_back(down);
    v.push_back(up);
    v.push_back(left);
    v.push_back(right);
    return v;
}

void MakeGrid(std::vector<std::vector<int>>& grid) 
{
    int n = grid.size();
    std::random_device rd;   
    std::mt19937 eng(rd());  
    std::uniform_int_distribution<int> distr(0, n);  

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            grid[i][j] = distr(eng) / ((n - 1));  
        }
    }
}

void PrintGrid(const std::vector<std::vector<int>>& grid) {
    int n = grid.size();
    std::cout << "Grid: " << std::endl;
    std::cout << "1. Points not considered ---> 0" << std::endl;
    std::cout << "2. Obstacles             ---> 1" << std::endl;
    std::cout << "3. Points considered     ---> 2" << std::endl;
    std::cout << "4. Points in final path  ---> 3" << std::endl;
    for (int j = 0; j < n; j++) {
        std::cout << "---";
    }
    std::cout << std::endl;

    for (const auto& row : grid) {
        for (const auto& ele : row) {
            if (ele == 3) {
                std::cout << GREEN << ele << RESET << " , ";
            }
            else if (ele == 1) {
                std::cout << RED << ele << RESET << " , ";
            }
            else if (ele == 2) {
                std::cout << BLUE << ele << RESET << " , ";
            }
            else {
                std::cout << ele << " , ";
            }
        }
        std::cout << std::endl << std::endl;
    }

    for (int j = 0; j < n; j++) {
        std::cout << "---";
    }
    std::cout << std::endl;
}

void PrintPath(std::vector<Node>& path_vector, const Node& start,
    const Node& goal, std::vector<std::vector<int>>& grid) {
    if (path_vector[0].id_ == -1) {
        std::cout << "No path exists" << std::endl;
        PrintGrid(grid);
        return;
    }
    std::cout << "Path (goal to start):" << std::endl;
    for (size_t i = 0; i < path_vector.size(); i++) {
        if (compareCoordinates(goal, path_vector[i])) {
            path_vector[i].PrintStatus();
            grid[path_vector[i].x_][path_vector[i].y_] = 3;
            while (path_vector[i].id_ != start.id_) {
                if (path_vector[i].id_ == path_vector[i].pid_) {
                    break;
                }
                for (size_t j = 0; j < path_vector.size(); j++) {
                    if (path_vector[i].pid_ == path_vector[j].id_) {
                        i = j;
                        path_vector[j].PrintStatus();
                        grid[path_vector[j].x_][path_vector[j].y_] = 3;
                    }
                }
            }
            break;
        }
    }
    grid[start.x_][start.y_] = 3;
    PrintGrid(grid);
}

void PrintCost(const std::vector<std::vector<int>>& grid,
    const std::vector<Node>& point_list) {
    int n = grid.size();
    std::vector<Node>::const_iterator it_v;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (it_v = point_list.begin(); it_v != point_list.end(); ++it_v) {
                if (i == it_v->x_ && j == it_v->y_) {
                    std::cout << std::setw(spacing_for_grid) << it_v->cost_ << " , ";
                    break;
                }
            }
            if (it_v == point_list.end()) {
                std::cout << std::setw(spacing_for_grid) << "  , ";
            }
        }
        std::cout << std::endl << std::endl;
    }
}

void PrintPathInOrder(const std::vector<Node>& path_vector,
    const Node& /*start*/, const Node& goal,
    std::vector<std::vector<int>>& grid) {
    if (path_vector[0].id_ == -1) {
        std::cout << "Path not found" << std::endl;
        PrintGrid(grid);
        return;
    }
    std::cout << "Path (goal to start):" << std::endl;
    size_t i = 0;
    while (!compareCoordinates(path_vector[i], goal)) {
        i++;
    }
    for (; i > 0; i = i - 1) {
        path_vector[i].PrintStatus();
        grid[path_vector[i].x_][path_vector[i].y_] = 3;
    }
    path_vector[0].PrintStatus();
    grid[path_vector[0].x_][path_vector[0].y_] = 3;
    PrintGrid(grid);
}
