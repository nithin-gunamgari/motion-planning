#include <vector>
#include <functional>
#include <set>

namespace AStar
{
    struct coor
    {
        int x, y;

        bool operator == (const coor& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(coor, coor)>;
    using CoordinateList = std::vector<coor>;

    struct Node
    {
        uint G, H;
        coor coordinates;
        Node *parent;

        Node(coor coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class maps
    {
        bool detectCollision(coor coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, coor coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        maps();
        void setWorldSize(coor worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(coor source_, coor target_);
        void addCollision(coor coordinates_);
        void removeCollision(coor coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        coor worldSize;
        uint directions;
    };

    class Heuristic
    {
        static coor getDelta(coor source_, coor target_);

    public:
        static uint manhattan(coor source_, coor target_);
        static uint euclidean(coor source_, coor target_);
        static uint octagonal(coor source_, coor target_);
    };
}
