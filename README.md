# Voronoi
![alt text](https://github.com/Wollnashorn/Voronoi/blob/master/voronoi_title.png)

### Example
```cpp
// Your Vector class must have the public floating-point fields x and y, that's the only requirement
using Voronoi = Voronoi<Vector2>;

// Any iterable container can be used as point source.
// The points must be of a type, that can be instantiated with two floats {x, y}
std::vector<Vector2> points = { {0, 0}, {2, 3}, {6, 1}, {3, 10} };
auto output = Voronoi::generate(points.cbegin(), points.cend());

// One way to iterate through all cells and edges
for (auto& cell : output.cells)
{
  auto incidentEdge = cell.incidentEdge;
  for (auto edge = incidentEdge; edge->next != incidentEdge; edge = edge->next)
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
  }
}

```
