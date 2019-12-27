#pragma once

#include <map>
#include <queue>

template <typename Vec> struct Voronoi
{
	using T = decltype(Vec::x);

	template <typename> struct Site;
	template <typename> struct SiteEvent;
	template <typename> struct SitePair;
	template <typename> struct Bisector;
	template <typename> struct CircleEvent;
	template <typename> struct Output;
	template <typename> struct Halfedge;
	template <typename> struct Cell;

	template <typename Iterator> using Beachline = std::map<SitePair<Iterator>, Bisector<Iterator>>;
	template <typename Iterator> using Sites = std::vector<Site<Iterator>>;
	template <typename Iterator> using SiteEvents = std::vector<SiteEvent<Iterator>>;
	template <typename Iterator> using CircleEvents = std::priority_queue<std::unique_ptr<CircleEvent<Iterator>>,
		std::vector<std::unique_ptr<CircleEvent<Iterator>>>, typename CircleEvent<Iterator>::Comparator>;
	template <typename Iterator> using Outputs = std::deque<Output<Iterator>>;

	// The sites are all allocated at the first step of the Voronoi generator and stay in a container
	template <typename Iterator> struct Site
	{
		Vec position;
		Iterator iterator;
		size_t index;
	};

	// Contains an iterator to the site that is going to be added
	template <typename Iterator> struct SiteEvent
	{
		typename Sites<Iterator>::iterator site;

		bool operator<(const SiteEvent& other) const
		{
			if (site->position.y == other.site->position.y)
				return site->position.x < other.site->position.x;
			return site->position.y < other.site->position.y;
		}
	};

	// Used as a key in the beach line tree structure
	template <typename Iterator> struct SitePair
	{
		typename Sites<Iterator>::iterator leftSite;
		typename Sites<Iterator>::iterator rightSite;

		bool operator<(const SitePair& other) const
		{
			const auto chooseNewer = [](const auto& left, const auto& right) -> const auto& {
				return left->index < right->index ? *right : *left;
			};

			// We compare the newer sites of both Site Pairs
			const auto& siteA = chooseNewer(leftSite, rightSite);
			const auto& siteB = chooseNewer(other.leftSite, other.rightSite);

			if (siteA.position.y < siteB.position.y)
			{
				return distanceComparison(*leftSite, *rightSite, siteB);
			}
			else if (siteB.position.y < siteA.position.y)
			{
				return !distanceComparison(*other.leftSite, *other.rightSite, siteA);
			}
			else if (siteA.index == siteB.index)
			{
				return leftSite->index < other.leftSite->index;
			}
			return siteA.index < siteB.index;
		}

	private:
		bool distanceComparison(const Site<Iterator>& left, const Site<Iterator>& right, const Site<Iterator>& newer) const
		{
			const auto distanceToArc = [&newer](const auto& site) {
				auto delta = newer.position - site.position;
				return (delta.x * delta.x + delta.y * delta.y) / (2 * delta.y);
			};

			if (left.position.y < right.position.y)
			{
				if (newer.position.x >= right.position.x)
					return true;
			}
			else if (left.position.y > right.position.y)
			{
				if (newer.position.x <= left.position.x)
					return false;
			}
			// if both sites share the same y-coordinate compare against the center of both sites
			else
			{
				return 2 * newer.position.x > left.position.x + right.position.x;
			}

			return distanceToArc(right) < distanceToArc(left);
		}
	};

	template <typename Iterator> struct CircleEvent
	{
		typename Beachline<Iterator>::iterator rightPair;
		Vec position;
		T radius;
		T eventPosition;
		bool active = true;
		unsigned index = 0u;

		struct Comparator
		{
			bool operator()(const std::unique_ptr<CircleEvent>& a, const std::unique_ptr<CircleEvent>& b)
			{
				if (a->eventPosition == b->eventPosition)
					return a->index > b->index;

				return a->eventPosition > b->eventPosition;
			}
		};
	};

	template <typename Iterator> struct Vertex
	{
		Vec circumcenter;
		T radius;
		std::array<Iterator, 3> triangle;
		Halfedge<Iterator>* incidentEdge = nullptr;
	};

	template <typename Iterator> struct Halfedge
	{
		Vertex<Iterator>* vertex = nullptr;
		Halfedge* twin = nullptr, * prev = nullptr, * next = nullptr;
		Cell<Iterator>* cell = nullptr;
	};

	template <typename Iterator> struct Cell
	{
		Iterator point;
		Halfedge<Iterator>* incidentEdge = nullptr;
	};

	template <typename Iterator> struct Output
	{
		std::vector<Vertex<Iterator>> vertices;
		std::vector<Halfedge<Iterator>> edges;
		std::vector<Cell<Iterator>> cells;
	};

	template <typename Iterator> struct Bisector
	{
		CircleEvent<Iterator>* circleEvent = nullptr;
		Halfedge<Iterator>* edge = nullptr;
	};

	template <typename Iterator>
	static Output<Iterator> generate(Iterator pointsBegin, Iterator pointsEnd)
	{
		Output<Iterator> output;

		const auto pointCount = std::distance(pointsBegin, pointsEnd);
		// At least three sites are required to form a Voronoi vertex
		if (pointCount < 3)
			return output;

		Sites<Iterator> sites;
		SiteEvents<Iterator> siteEvents;
		CircleEvents<Iterator> circleEvents;
		unsigned int circleEventIndex = 0u;


		// We allocate the final size before adding the sites to make sure the iterators will stay valid
		sites.reserve(pointCount);
		for (auto point = pointsBegin; point != pointsEnd; ++point)
		{
			sites.emplace_back(Site<Iterator>{ *point, point, 0 });
			siteEvents.emplace_back(SiteEvent<Iterator>{ --sites.end() });
		}


		// Site events will be added only once at the beginning, so we just have to sort it this time
		std::sort(siteEvents.begin(), siteEvents.end());

		auto siteEventIt = siteEvents.begin();
		for (size_t i = 0u; i < siteEvents.size(); ++i)
		{
			siteEventIt->site->index = i;
			++siteEventIt;
		}

		// We allocate the memory for the output containers too, as we know the maximum output beforehand
		output.cells.reserve(pointCount);
		output.vertices.reserve(2 * pointCount);
		output.edges.reserve(6 * pointCount);
		
		Beachline<Iterator> beachline;
		siteEventIt = siteEvents.begin();

		beachlineInitialization(siteEvents, beachline, siteEventIt, output);

		while (siteEventIt != siteEvents.end() || !circleEvents.empty())
		{
			if (circleEvents.empty())
			{
				processSiteEvent(beachline, circleEvents, siteEventIt, circleEventIndex, output);
			}
			else
			{
				if (siteEventIt == siteEvents.end())
				{
					auto circleEvent = *circleEvents.top();
					circleEvents.pop();
					processCircleEvent(beachline, circleEvents, circleEvent, output, circleEventIndex);
				}
				else
				{
					if (circleEvents.top()->eventPosition < siteEventIt->site->position.y)
					{
						auto circleEvent = *circleEvents.top();
						circleEvents.pop();
						processCircleEvent(beachline, circleEvents, circleEvent, output, circleEventIndex);
					}
					else
					{
						processSiteEvent(beachline, circleEvents, siteEventIt, circleEventIndex, output);
					}
				}
			}

			// Delete all inactive circle events
			while (!circleEvents.empty() && !circleEvents.top()->active)
				circleEvents.pop();
		}

		for (auto& cell : output.cells)
		{
			auto start = cell.incidentEdge;
			auto edge = start;
			while (edge->prev)
			{
				edge = edge->prev;

				if (edge == start)
					break;
			}

			// If the edge is part of an unclosed cell, we skip to the next one
			if (edge->prev)
				continue;

			auto rightEdge = start;
			while (rightEdge->next)
				rightEdge = rightEdge->next;

			edge->prev = rightEdge;
			rightEdge->next = edge;
		}

		return output;
	}

	template <typename Iterator>
	static void beachlineInitialization(SiteEvents<Iterator>& siteEvents, Beachline<Iterator>& beachline, 
		typename SiteEvents<Iterator>::iterator& siteEventIt, Output<Iterator>& output)
	{
		for (auto collinear = siteEvents.begin()+1; collinear != siteEvents.end() &&
			collinear->site->position.y == siteEvents.begin()->site->position.y; ++collinear)
		{
			auto firstSite = siteEventIt->site;
			auto secondSite = (++siteEventIt)->site;
			auto leftArc = addCollinearArc(beachline, firstSite, secondSite, beachline.end(), output);
		}

		if (siteEventIt == siteEvents.begin())
		{
			auto firstSite = siteEventIt->site;
			auto secondSite = (++siteEventIt)->site;
			addArc(beachline, firstSite, firstSite, secondSite, beachline.end(), output);
		}
		++siteEventIt;
	}

	template <typename Iterator>
	static void processSiteEvent(Beachline<Iterator>& beachline, CircleEvents<Iterator>& circleEvents, 
		typename SiteEvents<Iterator>::iterator& siteEventIt, unsigned int& circleEventIndex, Output<Iterator>& output)
	{
		auto site = siteEventIt->site;
		++siteEventIt;

		SitePair<Iterator> key = { site, site };
		auto rightPair = beachline.lower_bound(key);

		if (rightPair == beachline.end())
		{
			auto leftPair = rightPair;
			--leftPair;
			auto inserted = addArc(beachline, leftPair->first.rightSite, leftPair->first.rightSite, site, rightPair, output);
			activateCircleEvent(circleEvents, leftPair, inserted, circleEventIndex);
		}
		else if (rightPair == beachline.begin())
		{
			auto leftPair = addArc(beachline, rightPair->first.leftSite, rightPair->first.leftSite, site, rightPair, output);
			++leftPair;
			activateCircleEvent(circleEvents, leftPair, rightPair, circleEventIndex);
		}
		else
		{
			deactivateCircleEvent<Iterator>(rightPair);

			auto leftPair = rightPair;
			--leftPair;

			auto inserted = addArc(beachline, leftPair->first.rightSite, rightPair->first.leftSite, site, rightPair, output);

			// Check and activate circular event on the left
			activateCircleEvent(circleEvents, leftPair, inserted, circleEventIndex);
			// ... and on the right
			leftPair = ++inserted;
			activateCircleEvent(circleEvents, leftPair, rightPair, circleEventIndex);
		}
	}

	template <typename Iterator>
	static void processCircleEvent(Beachline<Iterator>& beachline, CircleEvents<Iterator>& circleEvents,
		const CircleEvent<Iterator>& circleEvent, Output<Iterator>& output, unsigned int& circleEventIndex)
	{
		auto rightPair = circleEvent.rightPair;
		auto leftPair = circleEvent.rightPair;
		--leftPair;

		auto right = rightPair->first.rightSite;
		auto middle = leftPair->first.rightSite;
		auto left = leftPair->first.leftSite;

		output.vertices.emplace_back(Vertex<Iterator>{ circleEvent.position, circleEvent.radius,
			{ left->iterator, middle->iterator, right->iterator } });
		auto vertex = &output.vertices.back();
		vertex->incidentEdge = rightPair->second.edge;

		output.edges.emplace_back(Halfedge<Iterator>{});
		auto& newEdge = output.edges.back();
		output.edges.emplace_back(Halfedge<Iterator>{});
		auto& newTwin = output.edges.back();

		newEdge.twin = &newTwin;
		newTwin.twin = &newEdge;

		newEdge.cell = &output.cells[left->index];
		newTwin.cell = &output.cells[right->index];

		leftPair->second.edge->vertex = vertex;
		rightPair->second.edge->vertex = vertex;
		newTwin.vertex = vertex;

		const auto connect = [](Halfedge<Iterator>* a, Halfedge<Iterator>* b) {
			a->next = b;
			b->prev = a;
		};

		connect(&newEdge, leftPair->second.edge);
		connect(leftPair->second.edge->twin, rightPair->second.edge);
		connect(rightPair->second.edge->twin, &newTwin);

		const_cast<SitePair<Iterator>&>(leftPair->first).rightSite = rightPair->first.rightSite;
		auto& bisector = leftPair->second;
		bisector.edge = &newEdge;

		beachline.erase(rightPair);
		rightPair = leftPair;

		if (leftPair != beachline.begin())
		{
			deactivateCircleEvent<Iterator>(leftPair);
			--leftPair;
			activateCircleEvent(circleEvents, leftPair, rightPair, circleEventIndex);
		}

		leftPair = rightPair;
		++rightPair;
		if (rightPair != beachline.end())
		{
			deactivateCircleEvent<Iterator>(rightPair);
			activateCircleEvent(circleEvents, leftPair, rightPair, circleEventIndex);
		}
	}

	template <typename Iterator>
	static typename Beachline<Iterator>::iterator addArc(Beachline<Iterator>& beachline, 
		typename Sites<Iterator>::iterator left, typename Sites<Iterator>::iterator right, 
		typename Sites<Iterator>::iterator added, typename Beachline<Iterator>::iterator hint, Output<Iterator>& output)
	{
		auto rightPair = beachline.insert(hint, std::make_pair(SitePair<Iterator>{ added, right }, Bisector<Iterator>{}));
		auto leftPair = beachline.insert(rightPair, std::make_pair(SitePair<Iterator>{ left, added }, Bisector<Iterator>{}));

		output.edges.push_back(Halfedge<Iterator>{});
		auto& leftEdge = output.edges.back();
		output.edges.push_back(Halfedge<Iterator>{});
		auto& rightEdge = output.edges.back();

		leftEdge.twin = &rightEdge;
		rightEdge.twin = &leftEdge;

		leftPair->second.edge = &leftEdge;
		rightPair->second.edge = &rightEdge;

		// We create the cell for the right site only during initialization
		if (output.cells.empty())
			output.cells.emplace_back(Cell<Iterator>{ right->iterator });
		output.cells.emplace_back(Cell<Iterator>{ added->iterator });

		leftEdge.cell = &output.cells[right->index];
		rightEdge.cell = &output.cells.back();

		leftEdge.cell->incidentEdge = &leftEdge;
		rightEdge.cell->incidentEdge = &rightEdge;

		return leftPair;
	}

	template <typename Iterator>
	static typename Beachline<Iterator>::iterator addCollinearArc(Beachline<Iterator>& beachline, 
		typename Sites<Iterator>::iterator right, typename Sites<Iterator>::iterator added, 
		typename Beachline<Iterator>::iterator hint, Output<Iterator>& output)
	{
		auto leftPair = beachline.insert(hint, std::make_pair(SitePair<Iterator>{ right, added }, Bisector<Iterator>{ }));

		output.edges.push_back(Halfedge<Iterator>{});
		auto& leftEdge = output.edges.back();
		output.edges.push_back(Halfedge<Iterator>{});
		auto& rightEdge = output.edges.back();

		leftEdge.twin = &rightEdge;
		rightEdge.twin = &leftEdge;
		leftPair->second.edge = &leftEdge;

		if (output.cells.empty())
			output.cells.emplace_back(Cell<Iterator>{ right->iterator });
		output.cells.emplace_back(Cell<Iterator>{ added->iterator });

		leftEdge.cell = &output.cells[right->index];
		rightEdge.cell = &output.cells.back();

		leftEdge.cell->incidentEdge = &leftEdge;
		rightEdge.cell->incidentEdge = &rightEdge;

		return leftPair;
	}

	template <typename Iterator>
	static void activateCircleEvent(CircleEvents<Iterator>& circleEvents, typename Beachline<Iterator>::iterator leftPair, 
		typename Beachline<Iterator>::iterator rightPair, unsigned int& circleEventIndex)
	{
		const auto euclideanDistance = [](const Vec& a, const Vec& b) {
			return hypot(a.x - b.x, a.y - b.y);
		};

		const auto perpendicular = [](const Vec& vec) {
			return Vec{ -vec.y, vec.x };
		};

		const auto& leftSite = leftPair->first.leftSite;
		const auto& middleSite = rightPair->first.leftSite;
		const auto& rightSite = rightPair->first.rightSite;

		auto directionLeft = perpendicular(middleSite->position - leftSite->position);
		auto directionRight = perpendicular(rightSite->position - middleSite->position);

		auto determinant = directionLeft.x * directionRight.y - directionLeft.y * directionRight.x;

		if (determinant > 0) //2 * std::numeric_limits<T>::epsilon())
		{
			auto originLeft = (leftSite->position + middleSite->position) / 2;
			auto originRight = (middleSite->position + rightSite->position) / 2;

			auto t = (originLeft.y * directionRight.x + directionRight.y * originRight.x -
				originRight.y * directionRight.x - directionRight.y * originLeft.x) / determinant;

			auto intersection = originLeft + directionLeft * t;
			auto radius = euclideanDistance(intersection, middleSite->position);

			auto circleEvent = std::make_unique<CircleEvent<Iterator>>(CircleEvent<Iterator>{ rightPair, intersection, radius });
			circleEvent->eventPosition = intersection.y + radius;
			circleEvent->index = circleEventIndex++;
			rightPair->second.circleEvent = circleEvent.get();
			circleEvents.emplace(std::move(circleEvent));
		}
	}

	template <typename Iterator>
	static void deactivateCircleEvent(typename Beachline<Iterator>::iterator beachlineNode)
	{
		if (beachlineNode->second.circleEvent)
		{
			beachlineNode->second.circleEvent->active = false;
			beachlineNode->second.circleEvent = nullptr;
		}
	}
};