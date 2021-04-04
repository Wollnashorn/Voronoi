# Voronoi
![alt text](https://github.com/Wollnashorn/Voronoi/blob/master/voronoi_title.png)

### Example
```cpp
// Your Vector class must have the public floating-point fields x and y, that's the only requirement
using Voronoi = Voronoi<Vector2>;

// Any iterable container can be used as point source.
// The points must be of a type, that can be instantiated with two floats {x, y}
std::vector<Vector2> points = {{0, 0}, {2, 3}, {6, 1}, {3, 10}};
auto output = Voronoi::generate(points.cbegin(), points.cend());

// Iterate through all Voronoi vertices and their corresponding Delaunay triangle
for (auto& vertex : output.vertices)
    auto [a, b, c] = vertex.triangle;

// One way to iterate through all cells and edges
for (auto& cell : output.cells)
{
    auto incidentEdge = cell.incidentEdge;
    auto edge = incidentEdge;
    do
    {
        // if the edge is infinite, the bisector can be calculated with the two neighbouring sites
        if (!edge->isFinite())
        {
            auto [rayOrigin, rayDirection] = edge->asRay();
        }
        // otherwise the edge has an associcated start and end vertex
        else
        {
            auto a = edge->vertex->circumcenter;
            auto b = edge->twin->vertex->circumcenter;
        }
        edge = edge->next;
    } while (edge != incidentEdge);
}
```

---


### Medial axis 
<img src="../assets/medialaxis.png" width="40%" align="right">
Voronoi diagrams can be used to find an approximated medial axis of a geometric shape. To do that, simply generate the diagram for the contour vertices. The more points are used to trace the contour, the more accurate the resulting medial axis will be.


For concave shapes it can be useful to ignore all Voronoi vertices when generating the diagram, which would lie outside of the shape.
That can be done with passing a predicate function like this.
```cpp
auto discardOutside = [&](const auto& voronoiVertex)
{
    auto sorted = voronoiVertex.triangle;
    std::sort(sorted.begin(), sorted.end());
    auto normalA = *sorted[1] - *sorted[0];
    auto normalB = *sorted[2] - *sorted[0];
    auto winding = normalA.x*normalB.y - normalA.y*normalB.x >= 0;
    return winding;
};

Voronoi::generate(outline.cbegin(), outline.cend(), discardOutside);
```
