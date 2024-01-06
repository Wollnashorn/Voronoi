#include <Voronoi.hpp>
#include <fstream>

template <class T> struct Vector { T x, y; };

int main()
{
    std::vector<Vector<float>> points = { {0, 0}, {2, 3}, {6, 1}, {3, 8}, {-10, 1}, {-6, 6}, {1, -1}, {-4, 8} };

    // The generator will use the given Vector<double> structure for the internal calculations.
    // In this example the points are casted to double precision floats - this is just for
    // demonstration, using single precision floats is numerically robust enough for most cases.
    auto diagram = Voronoi::generate<Vector<double>>(points.cbegin(), points.cend());

    // If no cast is desired:
    // auto diagram = Voronoi::generate(points.cbegin(), points.cend());

    std::ofstream svg{"diagram_simple.svg"};

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

        svg << R"(" fill="none" stroke="#5ed17d" stroke-width="0.1" />)";
    }

    for (auto& point : points)
        svg << R"(<circle cx=")" << point.x << R"(" cy=")" << point.y << R"(" r="0.15" fill="white" />)";

    svg << "</svg>\n";

    return 0;
}