# Voronoi
![alt text](https://github.com/Wollnashorn/Voronoi/blob/master/voronoi_title.png)

### Example
```cpp
// Your Vector class must have the public member variables x and y, that's the only requirement
using Voronoi = Voronoi<Vector2>;

// Any iterable container can be used as point source
std::vector<Vector2> points = { {0, 0}, {2, 3}, {6, 1}, {3, 10} };
auto output = Voronoi::generate(points.cbegin(), points.cend());

```
