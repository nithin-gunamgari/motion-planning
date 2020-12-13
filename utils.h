#include <vector>

#define BLACK "\x1b[1;30m"
#define RED "\x1b[1;31m"
#define GREEN "\x1b[1;32m"
#define YELLOW "\x1b[1;33m"
#define BLUE "\x1b[1;34m"
#define MAGENTA "\x1b[1;35m"
#define CYAN "\x1b[1;36m"
#define WHITE "\x1b[1;37m"
#define RESET "\x1b[1;0m"

class Node 
{
public:
    int x_;
    int y_;
    int id_;
    int pid_;
    double cost_;
    double h_cost_;

    Node(const int x = 0, const int y = 0, const double cost = 0,
        const double h_cost = 0, const int id = 0, const int pid = 0);

    void PrintStatus() const;

    Node operator+(const Node& p) const;
    Node operator-(const Node& p) const;
};

struct compare_cost 
{
    bool operator()(const Node& p1, const Node& p2) const;
};

std::vector<Node> GetMotion();

void PrintGrid(const std::vector<std::vector<int>>& grid);

void PrintPath(std::vector<Node>& path_vector, const Node& start_,
    const Node& goal_, std::vector<std::vector<int>>& grid);

void PrintCost(const std::vector<std::vector<int>>& grid,
    const std::vector<Node>& point_list);

void MakeGrid(std::vector<std::vector<int>>& grid);

void PrintPathInOrder(const std::vector<Node>& path_vector, const Node& start,
    const Node& goal, std::vector<std::vector<int>>& grid);

bool compareCoordinates(const Node& p1, const Node& p2);

bool checkOutsideBoundary(const Node& node, const int n);

