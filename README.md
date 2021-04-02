# Voronoi
![alt text](https://github.com/Wollnashorn/Voronoi/blob/master/voronoi_title.png)

### Example
```cpp
// Your Vector class must have the public member floating-point variables x and y, that's the only requirement
using Voronoi = Voronoi<Vector2>;

// Any iterable container can be used as point source.
// The points must be of a type, that can be casted to the provided Vector class
std::vector<Vector2> points = { {0, 0}, {2, 3}, {6, 1}, {3, 10} };
auto output = Voronoi::generate(points.cbegin(), points.cend());

// One way to iterate through all cells and edges
for (auto& cell : output.cells)
{
  auto incidentEdge = cell.incidentEdge;
  for (auto edge = incidentEdge; edge->next != incidentEdge; edge = edge->next)
  {
    // if the edge is infinite, it's direction can be calculated with their two associated cells
    if (!edge->isFinite())
    {
      auto [rayOrigin, rayDirection] = edge->asRay();
    }
  }
}

```
