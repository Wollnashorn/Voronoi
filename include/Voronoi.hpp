#pragma once

#include <cstddef>
#include <cmath>
#include <array>
#include <vector>
#include <map>
#include <queue>
#include <iterator>
#include <memory>
#include <memory_resource>
#include <algorithm>
#include <execution>

template <typename Vector> struct Voronoi
{
	using T = decltype(Vector::x);

	template <class> struct Site;
	template <class> struct SitePair;
	template <class> struct Bisector;
	template <class> struct Halfedge;
	template <class> struct Cell;

	template <class Iterator> using Beachline = std::pmr::map<SitePair<Iterator>, Bisector<Iterator>>;
	template <class Iterator> using Sites = std::vector<Site<Iterator>>;

	template <class Iterator> using BeachlineNode = typename Beachline<Iterator>::iterator;
	template <class Iterator> using SiteIterator = typename Sites<Iterator>::iterator;

	// The sites are all allocated at the first step of the Voronoi generator and stay in a sorted std::vector
	template <class Iterator> struct Site
	{
		Vector position;
		Iterator iterator;
		Cell<Iterator>* cell;
	};

	// Used as a key in the beachline binary tree structure, it contains the left and right site that form a bisector
	template <class Iterator> struct SitePair
	{
		SiteIterator<Iterator> leftSite;
		SiteIterator<Iterator> rightSite;

		constexpr bool operator<(const SitePair& other) const
		{
			const auto chooseNewer = [](const auto& left, const auto& right) -> const auto& {
				return right > left ? right : left;
			};

			// Compare the newer sites of both site pairs. For that we compare the memory addresses instead of 
			// the y-coordinate as this is slightly faster than comparing floating point numbers
			const auto siteA = chooseNewer(leftSite, rightSite);
			const auto siteB = chooseNewer(other.leftSite, other.rightSite);

			if (siteA->position.y < siteB->position.y)
				return distanceComparison(*leftSite, *rightSite, *siteB);
			else if (siteA->position.y > siteB->position.y)
				return !distanceComparison(*other.leftSite, *other.rightSite, *siteA);
			else if (siteA == siteB)
				return leftSite < other.leftSite;
			return siteA < siteB;
		}

	private:
		constexpr bool distanceComparison(const Site<Iterator>& left, const Site<Iterator>& right, const Site<Iterator>& newer) const
		{
			// If both sites share the same y-coordinate compare against the center of both sites
			if (left.position.y == right.position.y)
				return 2 * newer.position.x > left.position.x + right.position.x;

			if (left.position.y < right.position.y && newer.position.x >= right.position.x) return true;
			if (left.position.y > right.position.y && newer.position.x <= left.position.x) return false;

			auto squaredLength = [](const Vector& v) { return v.x * v.x + v.y * v.y; };
			const Vector deltaLeft = { newer.position.x - left.position.x, newer.position.y - left.position.y };
			const Vector deltaRight = { newer.position.x - right.position.x, newer.position.y - right.position.y };
			return squaredLength(deltaRight) * deltaLeft.y < squaredLength(deltaLeft) * deltaRight.y;
		}
	};

	template <class Iterator> struct CircleEvent
	{
		BeachlineNode<Iterator> rightNode;
		Vector position;
		T radius;
		T eventPosition;
		bool active = true;
		unsigned index = 0u;

		struct Comparator
		{
			constexpr bool operator()(const std::unique_ptr<CircleEvent<Iterator>>& a, const std::unique_ptr<CircleEvent<Iterator>>& b)
			{
				if (a->eventPosition == b->eventPosition) return a->index > b->index;
				return a->eventPosition > b->eventPosition;
			}
		};
	};

	template <class Iterator> struct CircleEvents
	{
		unsigned int indexCounter = 0;
		std::vector<std::unique_ptr<CircleEvent<Iterator>>> recycleBuffer;
		std::priority_queue<std::unique_ptr<CircleEvent<Iterator>>, std::vector<std::unique_ptr<CircleEvent<Iterator>>>,
			typename CircleEvent<Iterator>::Comparator> queue;
	};

	template <class Iterator> struct Vertex
	{
		Vector circumcenter;
		T radius;
		std::array<Iterator, 3> triangle;
		Halfedge<Iterator>* incidentEdge = nullptr;
	};

	struct Ray { Vector origin, direction; };

	template <class Iterator> struct Halfedge
	{
		Cell<Iterator>* cell = nullptr;
		Vertex<Iterator>* vertex = nullptr;
		Halfedge* twin = nullptr, *prev = nullptr, *next = nullptr;

		constexpr bool isFinite() const { return vertex && twin->vertex; }

		constexpr Ray asRay() const
		{
			if (vertex || twin->vertex)
			{
				auto primary = vertex ? this : twin;
				auto secondary = vertex ? twin : this;
				return { primary->vertex->circumcenter, perpendicular(*secondary->cell->point, *primary->cell->point) };
			}
			else
			{
				// When there is no Voronoi vertex at all, then this edge is a infinite straight with no intersections. 
				// In this case we output a ray with origin in the middle of both sites
				return Ray{ center(*cell->point, *twin->cell->point), perpendicular(*cell->point, *twin->cell->point) };
			}
		}
	};

	template <class Iterator> struct Cell
	{
		Iterator point;
		Halfedge<Iterator>* incidentEdge = nullptr;
	};

	template <class Iterator> struct Output
	{
		std::vector<Vertex<Iterator>> vertices;
		std::vector<Halfedge<Iterator>> edges;
		std::vector<Cell<Iterator>> cells;
	};

	template <class Iterator> struct Bisector
	{
		CircleEvent<Iterator>* circleEvent = nullptr;
		Halfedge<Iterator>* edge = nullptr;
	};

	template <class Iterator> static constexpr bool defaultIgnoreFunction(const Vertex<Iterator>&)
	{
		return false;
	}

	template <class Iterator>
	static Output<Iterator> generate(Iterator pointsBegin, Iterator pointsEnd)
	{
		return generate(pointsBegin, pointsEnd, defaultIgnoreFunction<Iterator>);
	}

	template <class Iterator, typename IgnoreFunction>
	static Output<Iterator> generate(Iterator pointsBegin, Iterator pointsEnd, const IgnoreFunction& ignoreFunction)
	{
		Output<Iterator> output;

		const auto pointCount = std::distance(pointsBegin, pointsEnd);
		// At least three sites are required to form a Voronoi vertex
		if (pointCount < 3)
			return output;

		//char buffer[sizeof(std::pair<SitePair<Iterator>, Bisector<Iterator>>) * 128];
		std::pmr::monotonic_buffer_resource bufferResource; //(buffer, sizeof(buffer));

		Sites<Iterator> sites;
		Beachline<Iterator> beachline(&bufferResource);
		CircleEvents<Iterator> circleEvents;

		// We allocate the final size before adding the sites to make sure the iterators will stay valid
		sites.reserve(pointCount);
		for (auto point = pointsBegin; point != pointsEnd; ++point)
			sites.emplace_back(Site<Iterator>{ Vector{ static_cast<T>(point->x), static_cast<T>(point->y) }, point, 0 });

		std::sort(std::execution::par_unseq, sites.begin(), sites.end(), [](const Site<Iterator>& a, const Site<Iterator>& b) {
			if (a.iterator->y == b.iterator->y) return a.iterator->x < b.iterator->x;
			return a.iterator->y < b.iterator->y;
		});

		// We allocate the memory for the output containers too, as we know the maximum output size beforehand
		output.cells.reserve(pointCount);
		output.vertices.reserve(2 * pointCount);
		output.edges.reserve(6 * pointCount);

		for (auto& site : sites)
		{
			output.cells.push_back(Cell<Iterator>{ site.iterator });
			site.cell = &output.cells.back();
		}

		auto siteEvent = sites.begin();
		beachlineInitialization(sites, beachline, siteEvent, output);

		while (siteEvent != sites.end() || !circleEvents.queue.empty())
		{
			// Always prioritize site events as they could deactivate a circle event that would otherwise has been processed
			if (siteEvent != sites.end() && (circleEvents.queue.empty() || siteEvent->position.y <= circleEvents.queue.top()->eventPosition))
				processSiteEvent(beachline, circleEvents, siteEvent, output);
			else
			{
				// We need to cast the processed circle event to a non-const reference in order to move it to the recycle buffer.
				// This is a workaround because std::priority_queue unfortunately doesn't allow us to pop and move the top element
				circleEvents.recycleBuffer.emplace_back(std::move(const_cast<std::unique_ptr<CircleEvent<Iterator>>&>(circleEvents.queue.top())));
				auto& circleEvent = *circleEvents.recycleBuffer.back();
				circleEvents.queue.pop();
				processCircleEvent(beachline, circleEvents, circleEvent, output, ignoreFunction);
			}

			// Skip and remove all incoming deactivated circle events
			while (!circleEvents.queue.empty() && !circleEvents.queue.top()->active)
			{
				circleEvents.recycleBuffer.emplace_back(std::move(const_cast<std::unique_ptr<CircleEvent<Iterator>>&>(circleEvents.queue.top())));
				circleEvents.queue.pop();
			}
		}

		// For infinite cells, link both infinite edges with each other so that each half edge has a successor and predecessor
		for (auto& cell : output.cells)
		{
			auto* start = cell.incidentEdge;
			auto* leftEdge = start;
			while (leftEdge->prev != start && leftEdge->prev) leftEdge = leftEdge->prev;

			// Skip this cell if it is already enclosed
			if (leftEdge->prev) continue;

			auto* rightEdge = start;
			while (rightEdge->next)	rightEdge = rightEdge->next;

			leftEdge->prev = rightEdge;
			rightEdge->next = leftEdge;
		}

		return output;
	}

	template <class Iterator>
	static constexpr void beachlineInitialization(Sites<Iterator>& sites, Beachline<Iterator>& beachline,
		SiteIterator<Iterator>& site, Output<Iterator>& output)
	{
		for (auto collinear = std::next(site); collinear != sites.end() && collinear->position.y == site->position.y; ++collinear)
		{
			auto firstSite = site;
			auto secondSite = ++site;
			auto leftArc = addCollinearArc(beachline, firstSite, secondSite, beachline.end(), output);
		}

		if (beachline.empty())
		{
			auto firstSite = site;
			auto secondSite = ++site;
			addArc(beachline, firstSite, firstSite, secondSite, beachline.end(), output);
		}
		++site;
	}

	template <class Iterator>
	static constexpr void processSiteEvent(Beachline<Iterator>& beachline, CircleEvents<Iterator>& circleEvents,
		SiteIterator<Iterator>& site, Output<Iterator>& output)
	{
		SitePair<Iterator> key = { site, site };
		auto rightNode = beachline.lower_bound(key);

		if (rightNode == beachline.end())
		{
			auto leftNode = std::prev(rightNode);
			auto inserted = addArc(beachline, leftNode->first.rightSite, leftNode->first.rightSite, site, rightNode, output);
			activateCircleEvent( circleEvents, leftNode, inserted);
		}
		else if (rightNode == beachline.begin())
		{
			auto leftNode = addArc(beachline, rightNode->first.leftSite, rightNode->first.leftSite, site, rightNode, output);
			activateCircleEvent(circleEvents, ++leftNode, rightNode);
		}
		else
		{
			deactivateCircleEvent<Iterator>(rightNode);
			auto leftNode = std::prev(rightNode);
			auto inserted = addArc(beachline, leftNode->first.rightSite, rightNode->first.leftSite, site, rightNode, output);

			// Check and activate circular event on the left
			activateCircleEvent(circleEvents, leftNode, inserted);
			// ... and on the right
			activateCircleEvent(circleEvents, ++inserted, rightNode);
		}
		++site;
	}

	template <class Iterator, class IgnoreFunction>
	static constexpr void processCircleEvent(Beachline<Iterator>& beachline, CircleEvents<Iterator>& circleEvents,
		const CircleEvent<Iterator>& circleEvent, Output<Iterator>& output, IgnoreFunction ignoreFunction = defaultIgnoreFunction<Iterator>)
	{
		auto rightNode = circleEvent.rightNode;
		auto leftNode = std::prev(circleEvent.rightNode);

		auto right = rightNode->first.rightSite;
		auto middle = leftNode->first.rightSite;
		auto left = leftNode->first.leftSite;

		auto& newEdge = (output.edges.emplace_back(Halfedge<Iterator>{ left->cell }), output.edges.back());
		auto& newTwin = (output.edges.emplace_back(Halfedge<Iterator>{ right->cell }), output.edges.back());

		newEdge.twin = &newTwin;
		newTwin.twin = &newEdge;

		auto triangle = std::array<Iterator, 3>{ left->iterator, middle->iterator, right->iterator };
		auto vertex = Vertex<Iterator>{ circleEvent.position, circleEvent.radius, std::move(triangle) };
		auto ignore = ignoreFunction(vertex);

		if (!ignore)
		{
			vertex.incidentEdge = rightNode->second.edge;
			output.vertices.emplace_back(std::move(vertex));
			leftNode->second.edge->vertex = rightNode->second.edge->vertex = newTwin.vertex = &output.vertices.back();
		}

		auto connect = [](Halfedge<Iterator>* a, Halfedge<Iterator>* b) {
			a->next = b;
			b->prev = a;
		};

		connect(&newEdge, leftNode->second.edge);
		connect(leftNode->second.edge->twin, rightNode->second.edge);
		connect(rightNode->second.edge->twin, &newTwin);

		auto& bisector = leftNode->second;
		bisector.edge = &newEdge;

		// We cast here to a non-const reference once again, as we want to remove the inbetween bisector node
		// and we know that the sorting order will not change. AB CD -> AD
		const_cast<SitePair<Iterator>&>(leftNode->first).rightSite = rightNode->first.rightSite;
		beachline.erase(rightNode);

		rightNode = leftNode;
		if (leftNode != beachline.begin())
		{
			--leftNode;
			deactivateCircleEvent<Iterator>(rightNode);
			activateCircleEvent(circleEvents, leftNode, rightNode);
		}

		leftNode = rightNode;
		++rightNode;
		if (rightNode != beachline.end())
		{
			deactivateCircleEvent<Iterator>(rightNode);
			activateCircleEvent(circleEvents, leftNode, rightNode);
		}
	}

	template <class Iterator>
	static constexpr BeachlineNode<Iterator> addArc(Beachline<Iterator>& beachline,	SiteIterator<Iterator> left, 
		SiteIterator<Iterator> right, SiteIterator<Iterator> added, BeachlineNode<Iterator> hint, Output<Iterator>& output)
	{
		auto rightNode = beachline.insert(hint, { SitePair<Iterator>{ added, right }, Bisector<Iterator>{} });
		auto leftNode = beachline.insert(rightNode, { SitePair<Iterator>{ left, added }, Bisector<Iterator>{} });

		auto& leftEdge = (output.edges.push_back(Halfedge<Iterator>{ right->cell }), output.edges.back());
		auto& rightEdge = (output.edges.push_back(Halfedge<Iterator>{ added->cell }), output.edges.back());

		leftEdge.twin = &rightEdge;
		rightEdge.twin = &leftEdge;
		leftEdge.cell->incidentEdge = &leftEdge;
		rightEdge.cell->incidentEdge = &rightEdge;
		leftNode->second.edge = &leftEdge;
		rightNode->second.edge = &rightEdge;

		return leftNode;
	}

	template <class Iterator>
	static constexpr BeachlineNode<Iterator> addCollinearArc(Beachline<Iterator>& beachline, 
		SiteIterator<Iterator> right, SiteIterator<Iterator> added, BeachlineNode<Iterator> hint, Output<Iterator>& output)
	{
		auto leftNode = beachline.insert(hint, { SitePair<Iterator>{ right, added }, Bisector<Iterator>{} });

		auto& leftEdge = (output.edges.push_back(Halfedge<Iterator>{ right->cell }), output.edges.back());
		auto& rightEdge = (output.edges.push_back(Halfedge<Iterator>{ added->cell }), output.edges.back());

		leftEdge.twin = &rightEdge;
		rightEdge.twin = &leftEdge;
		leftEdge.cell->incidentEdge = &leftEdge;
		rightEdge.cell->incidentEdge = &rightEdge;
		leftNode->second.edge = &leftEdge;

		return leftNode;
	}

	template <typename Iterator>
	static constexpr void activateCircleEvent(CircleEvents<Iterator>& circleEvents,	BeachlineNode<Iterator> leftNode, 
		BeachlineNode<Iterator> rightNode)
	{
		auto euclideanDistance = [](const Vector& a, const Vector& b) { return std::hypot(a.x - b.x, a.y - b.y); };

		const auto& leftSite = leftNode->first.leftSite;
		const auto& middleSite = rightNode->first.leftSite;
		const auto& rightSite = rightNode->first.rightSite;

		Vector directionLeft = perpendicular(leftSite->position, middleSite->position);
		Vector directionRight = perpendicular(middleSite->position, rightSite->position);

		// Only add a new circle event if the two bisector edges will intersect in the future. This is not the case if they are parallel
		auto determinant = directionLeft.x * directionRight.y - directionLeft.y * directionRight.x;
		if (determinant > 0)
		{
			Vector originLeft = center(leftSite->position, middleSite->position);
			Vector originRight = center(middleSite->position, rightSite->position);

			auto t = (originLeft.y * directionRight.x - directionRight.y * originLeft.x + 
				directionRight.y * originRight.x - originRight.y * directionRight.x) / determinant;

			Vector intersection = { originLeft.x + directionLeft.x * t, originLeft.y + directionLeft.y * t };
			auto radius = euclideanDistance(intersection, middleSite->position);

			auto circleEvent = circleEvents.recycleBuffer.empty()
				? std::make_unique<CircleEvent<Iterator>>()
				: std::move(circleEvents.recycleBuffer.back());

			if (!circleEvents.recycleBuffer.empty()) circleEvents.recycleBuffer.pop_back();

			*circleEvent = { rightNode, intersection, radius };
			circleEvent->eventPosition = intersection.y + radius;
			circleEvent->index = circleEvents.indexCounter++;
			rightNode->second.circleEvent = circleEvent.get();
			circleEvents.queue.emplace(std::move(circleEvent));
		}
	}

	template <class Iterator>
	static constexpr void deactivateCircleEvent(BeachlineNode<Iterator> beachlineNode)
	{
		if (beachlineNode->second.circleEvent)
		{
			beachlineNode->second.circleEvent->active = false;
			beachlineNode->second.circleEvent = nullptr;
		}
	}

protected:
	template <class InputVec> static constexpr Vector center(const InputVec& a, const InputVec& b) {
		return { (a.x + b.x) / 2, (a.y + b.y) / 2 };
	}

	template <class InputVec> static constexpr Vector perpendicular(const InputVec& a, const InputVec& b) {
		return { a.y - b.y, b.x - a.x };
	}
};