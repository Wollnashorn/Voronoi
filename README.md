# Voronoi


<a href="https://wollnashorn.github.io/Voronoi/"><img src="../assets/example_europe_header.webp"></a>

---

Generates a Voronoi diagram with the associated Delaunay triangulation for a set of 2D points using Fortune's algorithm.

<!-- - Single header file - requires at least C++17
- The resulting diagram is in form of a [Half-Edge Data Structure](https://en.wikipedia.org/wiki/Doubly_connected_edge_list)
- Stability is alright for normal use cases, edge cases like parallel points are handled properly
- Duplicate points are filtered, there is no need to remove doubles in the point input
- Building a diagram for 100,000 points takes only around 70ms on a Ryzen 5600X
- There are some examples provided, which are hopefully self-explanatory -->

- **Single Header**: Just one small header file to include - requires at least C++17
- **Half-Edge Data Structure**: The resulting Voronoi diagram is represented using a [Half-Edge Data Structure](https://en.wikipedia.org/wiki/Doubly_connected_edge_list)
- **Stability**: Robust for typical use cases with proper handling of edge scenarios such as parallel points
- **Duplicate Point Handling**: No need to pre-process input points - duplicate points are automatically filtered
- **Performance**: Building a diagram for 100,000 points takes just ~70ms on a Ryzen 5600X
- **Example Usage**: There are some examples provided, which are hopefully self-explanatory

### Example
```cpp
// Your vector class must have the public floating-point fields x and y, that's the only requirement
struct Vector2 { float x, y; };

// Any iterable container can be used as point source.
// The points must be of a type, that can be instantiated with two floats {x, y}
std::vector<Vector2> points = {{0, 0}, {2, 3}, {6, 1}, {3, 10}};
auto output = Voronoi::generate(points.begin(), points.end());

// Iterate through all Voronoi vertices and their corresponding Delaunay triangle
for (auto& vertex : output.vertices)
{
    auto circumcenter = vertex.circumcenter;
    auto radius = vertex.radius;
    auto [a, b, c] = vertex.triangle;
}

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
Voronoi diagrams can be used to find an approximated medial axis of a geometric shape. To do that, simply generate the diagram of the contour vertices. The more points are used to trace the contour, the more accurate the resulting medial axis will be.

For concave shapes it can be useful to ignore all Voronoi vertices when generating the diagram, which would lie outside of the shape.
That can be done with passing a predicate function like this:
```cpp
auto discardOutside = [](const auto& voronoiVertex)
{
    auto sorted = voronoiVertex.triangle;
    std::sort(sorted.begin(), sorted.end());
    auto normalA = *sorted[1] - *sorted[0];
    auto normalB = *sorted[2] - *sorted[0];
    return normalA.x*normalB.y - normalA.y*normalB.x >= 0;
};

Voronoi::generate(outlineBegin, outlineEnd, discardOutside);
```
> Here resulting triangles having the wrong winding order will be discarded 
