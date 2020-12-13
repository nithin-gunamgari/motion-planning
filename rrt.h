#include <limits>
#include "utils.h"

class RRT 
{
public:
   
    Node FindNearestPoint(Node& new_node) const;
    bool CheckObstacle(const Node& n_1, const Node& n_2) const;
    static Node GenerateRandomNode(const int n);
    std::vector<Node> rrt
    (
        std::vector<std::vector<int>>& grid, const Node& start_in,
        const Node& goal_in, const int max_iter_x_factor = 500,
        const double threshold_in = std::numeric_limits<double>::infinity()
    );

    bool CheckGoalVisible(const Node& new_node);
    void CreateObstacleList(std::vector<std::vector<int>>& grid);

private:
    std::vector<Node> point_list_;
    std::vector<Node> obstacle_list_;
    Node start_, goal_;
    double threshold_ = 1;
    int n = 0;
};
