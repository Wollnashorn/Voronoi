#include "Voronoi.hpp"
#include <iostream>

template <typename T> struct Vector {
	T x, y;

	friend std::ostream& operator <<(std::ostream& output, const Vector<T>& vector) {
		return output << vector.x << ", " << vector.y;
	}
};

int main()
{
	const std::vector<Vector<float>> points = { {0, 0}, {2, 3}, {6, 1}, {3, 10} };
	auto output = Voronoi<Vector<double>>::generate(points.cbegin(), points.cend());

	std::cout << "Cells generated: " << output.cells.size() << std::endl;

	auto cellIndex = 0u;
	for (auto& cell : output.cells)
	{
		std::cout << "Cell " << cellIndex++ << std::endl;
		auto incidentEdge = cell.incidentEdge;
		auto edge = incidentEdge;
		do {
			if (edge->isFinite())
				std::cout << "Edge: (" << edge->vertex->circumcenter << ") to (" << edge->twin->vertex->circumcenter << ") " << std::endl;
			else
			{
				auto ray = edge->asRay();
				std::cout << "Ray: (" << ray.origin << ") with direction (" << ray.direction << ")" << std::endl; 
			}
			edge = edge->next;
		} while (edge != incidentEdge);
	}

	return 0;
}