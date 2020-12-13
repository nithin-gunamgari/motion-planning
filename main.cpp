#include <iostream>
#include "AStar.hpp"

int main()
{
    AStar::maps generator;
    int r, c, w, wr, wc, sr, sc, dr, dc;

    std::cout << "Enter number of row of World :";
    std::cin >> r;

    std::cout << "Enter number of column of World :";
    std::cin >> c;

    generator.setWorldSize({r, c});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::cout << "Enter number of Obstacles in World :";
    std::cin >> w;
    
    for (int i = 0; i < w; i++) 
    {
        std::cout << "Enter positon of Obstacles in World :";
        std::cin >> wr>>wc;
    
        generator.addCollision({ wr,wc });
    }
    
    std::cout << "Enter positon of Source in World :";
    std::cin >> sr >> sc;

    std::cout << "Enter positon of Destination in World :";
    std::cin >> dr >> dc;

    auto path = generator.findPath({sr, sc}, {dr, dc});
    auto v = path[0];

    if (v.x != dr || v.y != dc) 
    {
        std::cout << "Path not found";
    }

    else 
    {
        std::cout << "Generate path ... \n";

        for (auto& coordinate : path)
        {
            std::cout << coordinate.x << " " << coordinate.y << "\n";
        }
    }
    
}