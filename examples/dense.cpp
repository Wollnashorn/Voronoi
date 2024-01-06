#include <Voronoi.hpp>
#include <fstream>

#include <iostream>
#include <chrono>

static constexpr size_t count = 10'000;

template <class T> struct Vector { T x, y; };

int main()
{
    std::array<Vector<float>, count> points;
    for (auto& point : points)
        point = {-10 + 20 * static_cast<float>(rand()) / RAND_MAX,
                 -10 + 20 * static_cast<float>(rand()) / RAND_MAX};

    auto start = std::chrono::high_resolution_clock::now();
    auto diagram = Voronoi::generate(points.cbegin(), points.cend());
    auto duration = std::chrono::high_resolution_clock::now() - start;

    std::cout << "Diagram of " << points.size() << " points generated in " << 
        std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << "ms\n";

    std::ofstream svg{"diagram_dense.svg"};

    svg << R"(<?xml version="1.0" encoding="UTF-8"?>)";
    svg << R"(<svg xmlns="http://www.w3.org/2000/svg" version="1.1" width="100%" height="100%" )"
           R"(viewBox="-10 -10 20 20" style="background-color: #1e1e1e">)";

    for (auto& cell : diagram.cells)
    {
        svg << R"(<polygon points=")";

        auto incidentEdge = cell.incidentEdge;
        auto edge = incidentEdge;
        do {
            if (edge->vertex)
                svg << edge->vertex->circumcenter.x << ", " << edge->vertex->circumcenter.y << " ";
            
            if (!edge->isFinite())
            {
                auto ray = edge->asRay();
                svg << ray.origin.x + 100 * ray.direction.x << ", " << ray.origin.y + 100 * ray.direction.y << " ";
            }
            edge = edge->next;
        } while (edge != incidentEdge);

        svg << R"(" fill="none" stroke="#5ed17d" stroke-width="0.01" />)";
    }

    for (auto& point : points)
        svg << R"(<circle cx=")" << point.x << R"(" cy=")" << point.y << R"(" r="0.015" fill="white" />)";

    svg << "</svg>\n";

    return 0;
}