#include <cmath>
#include <random>
#include "rrt.h"

constexpr double half_grid_unit = 0.5;
constexpr double tol_l_limit = 0.000001;

Node RRT::FindNearestPoint(Node& new_node) const {
    Node nearest_node(-1, -1, -1, -1, -1, -1);
    std::vector<Node>::const_iterator it_v;
    std::vector<Node>::const_iterator it_v_store;
    // use just distance not total cost
    auto dist = static_cast<double>(n * n);
    for (it_v = point_list_.begin(); it_v != point_list_.end(); ++it_v) {
        auto new_dist = static_cast<double>(
            std::sqrt(static_cast<double>(it_v->x_ - new_node.x_) *
                static_cast<double>(it_v->x_ - new_node.x_) +
                static_cast<double>(it_v->y_ - new_node.y_) *
                static_cast<double>(it_v->y_ - new_node.y_)));
        if (new_dist > threshold_) {
            continue;
        }
        if (CheckObstacle(*it_v, new_node)) {
            continue;
        }
        if (it_v->id_ == new_node.id_) {
            continue;
        }
        if (it_v->pid_ == new_node.id_) {
            continue;
        }
        if (new_dist >= dist) {
            continue;
        }
        dist = new_dist;
        it_v_store = it_v;
    }
    if (dist != n * n) {
        nearest_node = *it_v_store;
        new_node.pid_ = nearest_node.id_;
        new_node.cost_ = nearest_node.cost_ + dist;
    }
    return nearest_node;
}

bool RRT::CheckObstacle(const Node& n_1, const Node& n_2) const {
    if (n_2.y_ - n_1.y_ == 0) {
        double c = n_2.y_;
        for (const auto& obs_node : obstacle_list_) {
            if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
                ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
                continue;
            }
            if (static_cast<double>(obs_node.y_) == c) {
                return true;
            }
        }
    }
    else {
        double slope = static_cast<double>(n_2.x_ - n_1.x_) /
            static_cast<double>(n_2.y_ - n_1.y_);
        double c =
            static_cast<double>(n_2.x_) - slope * static_cast<double>(n_2.y_);
        for (const auto& obs_node : obstacle_list_) {
            if (!(((n_1.y_ >= obs_node.y_) && (obs_node.y_ >= n_2.y_)) ||
                ((n_1.y_ <= obs_node.y_) && (obs_node.y_ <= n_2.y_)))) {
                continue;
            }
            if (!(((n_1.x_ >= obs_node.x_) && (obs_node.x_ >= n_2.x_)) ||
                ((n_1.x_ <= obs_node.x_) && (obs_node.x_ <= n_2.x_)))) {
                continue;
            }
            std::vector<double> arr(4);

            arr[0] = static_cast<double>(obs_node.x_) + half_grid_unit -
                slope * (static_cast<double>(obs_node.y_) + half_grid_unit) - c;
            arr[1] = static_cast<double>(obs_node.x_) + half_grid_unit -
                slope * (static_cast<double>(obs_node.y_) - half_grid_unit) - c;
            arr[2] = static_cast<double>(obs_node.x_) - half_grid_unit -
                slope * (static_cast<double>(obs_node.y_) + half_grid_unit) - c;
            arr[3] = static_cast<double>(obs_node.x_) - half_grid_unit -
                slope * (static_cast<double>(obs_node.y_) - half_grid_unit) - c;
            double count = 0;
            for (auto& a : arr) {
                if (std::fabs(a) <= tol_l_limit) {
                    a = 0;
                }
                else {
                    count += a / std::fabs(a);
                }
            }
            if (std::abs(count) < 3) {
                return true;
            }
        }
    }
    return false;
}

Node RRT::GenerateRandomNode(const int n) {
    std::random_device rd;  
    std::mt19937 eng(rd()); 
    std::uniform_int_distribution<int> distr(0, n - 1);  
    int x = distr(eng);
    int y = distr(eng);
    Node new_node(x, y, 0, 0, n * x + y, 0);
    return new_node;
}

std::vector<Node> RRT::rrt(std::vector<std::vector<int>>& grid,
    const Node& start_in, const Node& goal_in,
    const int max_iter_x_factor,
    const double threshold_in) {
    start_ = start_in;
    goal_ = goal_in;
    n = grid.size();
    threshold_ = threshold_in;
    int max_iter = max_iter_x_factor * n * n;
    CreateObstacleList(grid);
    point_list_.push_back(start_);
    Node new_node = start_;
    grid[start_.x_][start_.y_] = 2;
    if (CheckGoalVisible(new_node)) {
        return this->point_list_;
    }
    int iter = 0;
    while (iter <= max_iter) {
        iter++;
        new_node = GenerateRandomNode(n);
        if (grid[new_node.x_][new_node.y_] != 0) {
            continue;
        }
        Node nearest_node = FindNearestPoint(new_node);
        if (nearest_node.id_ == -1) {
            continue;
        }
        grid[new_node.x_][new_node.y_] = 2;
        point_list_.push_back(new_node);
        if (CheckGoalVisible(new_node)) {
            return this->point_list_;
        }
    }
    Node no_path_node(-1, -1, -1, -1, -1, -1);
    point_list_.clear();
    point_list_.push_back(no_path_node);
    return point_list_;
}

bool RRT::CheckGoalVisible(const Node& new_node) {
    if (!CheckObstacle(new_node, goal_)) {
        auto new_dist = static_cast<double>(
            std::sqrt(static_cast<double>(goal_.x_ - new_node.x_) *
                static_cast<double>(goal_.x_ - new_node.x_) +
                static_cast<double>(goal_.y_ - new_node.y_) *
                static_cast<double>(goal_.y_ - new_node.y_)));
        if (new_dist <= threshold_) {
            goal_.pid_ = new_node.id_;
            goal_.cost_ = new_dist + new_node.cost_;
            point_list_.push_back(goal_);
            return true;
        }
    }
    return false;
}

void RRT::CreateObstacleList(std::vector<std::vector<int>>& grid) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (grid[i][j] == 1) {
                Node obs(i, j, 0, 0, i * n + j, 0);
                obstacle_list_.push_back(obs);
            }
        }
    }
}


