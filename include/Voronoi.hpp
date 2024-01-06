#pragma once

#include <cmath>
#include <array>
#include <vector>
#include <deque>
#include <set>
#include <algorithm>

namespace Voronoi
{
    constexpr auto defaultIgnorePredicate = [](auto&&) { return false; };

    template <class T> struct SparseAllocator
    {
        using size_type = size_t;
        using difference_type = ptrdiff_t;
        using pointer = T*;
        using const_pointer = const T*;
        using reference = T&;
        using const_reference = const T&;
        using value_type = T;

        std::deque<std::array<std::byte, sizeof(T)>> memory;
        std::vector<T*> recycle;

        pointer allocate(size_type)
        {
            if (!recycle.empty())
            {
                auto recycled = recycle.back();
                recycle.pop_back();
                return recycled;
            }
            else return reinterpret_cast<pointer>(&memory.emplace_back());
        }
        void deallocate(pointer pointer, size_type) { recycle.push_back(pointer); }
    };

    template <class Vector, class Iterator> struct Diagram
    {
        struct Bisector;
        struct Site;
        struct Cell;
        struct Halfedge;
        struct CircleEvent;

        using Floating = decltype(Vector::x);
        using Sites = std::vector<Site>;
        using SiteIterator = typename Sites::iterator;

        using Beachline = std::set<Bisector, std::less<>, SparseAllocator<Bisector>>;
        using BeachlineIterator = typename Beachline::iterator;

        struct Vertex
        {
            Vector circumcenter;
            Floating radius;
            std::array<Iterator, 3> triangle;
            Halfedge* incidentEdge = nullptr;
        };

        struct Ray { Vector origin, direction; };

        struct Halfedge
        {
            Cell* cell = nullptr;
            Vertex* vertex = nullptr;
            Halfedge* twin = nullptr, *prev = nullptr, *next = nullptr;

            Halfedge() = default;
            Halfedge(Cell* cell) : cell(cell) {}

            constexpr bool isFinite() const { return vertex && twin->vertex; }
            constexpr Ray asRay() const
            {
                auto center = [](const auto& a, const auto& b) { return Vector{ (a.x + b.x) / 2, (a.y + b.y) / 2 }; };
                auto perpendicular = [](const auto& a, const auto& b) { return Vector{ a.y - b.y, b.x - a.x }; };
                if (vertex || twin->vertex)
                {
                    auto primary = vertex ? this : twin;
                    auto secondary = vertex ? twin : this;
                    return { primary->vertex->circumcenter, perpendicular(*secondary->cell->point, *primary->cell->point) };
                }
                else
                    // When there is no Voronoi vertex at all, this edge is a infinite straight with no intersections. 
                    // In this case we output a ray with origin in the middle of both sites
                    return Ray{ center(*cell->point, *twin->cell->point), perpendicular(*cell->point, *twin->cell->point) };
            }
        };

        struct Cell
        {
            Iterator point;
            Halfedge* incidentEdge = nullptr;
        };

        struct Site
        {
            Iterator iterator;
            Vector position;
            Cell* cell = nullptr;
        };

        struct CircleEvent
        {
            BeachlineIterator node;
            Vector position;
            Floating radius;
            Floating eventPosition;
            size_t index = 0;
            bool active = true;

            CircleEvent(BeachlineIterator node, const Vector& position, Floating radius, Floating eventPosition, size_t index)
                : node(node), position(position), radius(radius), eventPosition(eventPosition), index(index) {}
        };

        struct Bisector
        {
            SiteIterator leftSite = nullptr;
            mutable SiteIterator rightSite = nullptr;
            mutable CircleEvent* circleEvent = nullptr;
            mutable Halfedge* edge = nullptr;

            constexpr bool operator<(const Bisector& other) const
            {
                auto chooseNewer = [](const auto& left, const auto& right) -> const auto& {
                    return right > left ? right : left;
                };

                auto distanceComparison = [](const auto& left, const auto& right, const auto& newer)
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
        };

        struct CircleEventQueue
        {
            std::deque<CircleEvent> circleEventsBuffer;
            std::vector<CircleEvent*> recycleBuffer;
            std::vector<CircleEvent*> queue;

            static constexpr auto compare = [](const CircleEvent* a, const CircleEvent* b) {
                if (a->eventPosition == b->eventPosition) return a->index > b->index;
                return a->eventPosition > b->eventPosition;
            };

            template <class... Args> constexpr CircleEvent& emplace(Args&&... args) noexcept
            {
                auto& circleEvent = recycleBuffer.empty()
                                  ? circleEventsBuffer.emplace_back(std::forward<Args>(args)...)
                                  : *recycleBuffer.back() = { std::forward<Args>(args)... };
                queue.push_back(&circleEvent);
                std::push_heap(queue.begin(), queue.end(), compare);
                if (!recycleBuffer.empty()) recycleBuffer.pop_back();
                return circleEvent;
            }
            
            constexpr CircleEvent& pop() noexcept
            {
                std::pop_heap(queue.begin(), queue.end(), compare);
                recycleBuffer.push_back(queue.back());
                queue.pop_back();
                return *recycleBuffer.back();
            }
        };

        std::vector<Vertex> vertices;
        std::vector<Halfedge> edges;
        std::vector<Cell> cells;

        template <class IgnorePredicate> Diagram(Iterator pointsBegin, Iterator pointsEnd, IgnorePredicate&& ignorePredicate)
        {
            const auto pointCount = std::distance(pointsBegin, pointsEnd);

            Beachline beachline;
            CircleEventQueue circleEvents;

            std::vector<Site> sites;
            sites.reserve(pointCount);
            for (auto point = pointsBegin; point != pointsEnd; ++point)
                sites.push_back({ point, Vector{ static_cast<Floating>(point->x), static_cast<Floating>(point->y) } });

            std::sort(sites.begin(), sites.end(), [](const Site& a, const Site& b) {
                if (a.position.y == b.position.y) return a.position.x < b.position.x;
                return a.position.y < b.position.y;
            });

            // Allocate the memory for the output containers too, as we know the maximum output size beforehand
            cells.reserve(pointCount);
            vertices.reserve(2 * pointCount);
            edges.reserve(6 * pointCount);

            for (auto site = sites.begin(); site != sites.end(); ++site)
            {
                // Filter out duplicate sites
                auto nextSite = std::next(site);
                if (nextSite != sites.end() && nextSite->position.y == site->position.y && nextSite->position.x == site->position.x)
                    continue;
                cells.push_back({ site->iterator });
                site->cell = &cells.back();
            }
            auto site = sites.begin();
            size_t circleEventCounter = 0;

            auto addEdgeTwins = [&](auto leftCell, auto rightCell) -> std::pair<Halfedge&, Halfedge&> {
                auto& leftEdge = edges.emplace_back(leftCell);
                auto& rightEdge = edges.emplace_back(rightCell);
                leftEdge.twin = &rightEdge;				rightEdge.twin = &leftEdge;
                leftCell->incidentEdge = &leftEdge;		rightCell->incidentEdge = &rightEdge;
                return { leftEdge, rightEdge };
            };

            auto addCollinearArc = [&](auto left, auto added, auto hint) {
                auto leftNode = beachline.emplace_hint(hint, Bisector{ left, added });
                leftNode->edge = &addEdgeTwins(left->cell, added->cell).first;
                return leftNode;
            };

            auto addArc = [&](auto left, auto right, auto added, auto hint) {
                auto rightNode = beachline.emplace_hint(hint, Bisector{ added, right });
                auto leftNode = beachline.emplace_hint(rightNode, Bisector{ left, added });
                auto edges = addEdgeTwins(right->cell, added->cell);
                leftNode->edge = &edges.first; rightNode->edge = &edges.second;
                return leftNode;
            };

            auto center = [](const auto& a, const auto& b) { return Vector{ (a.x + b.x) / 2, (a.y + b.y) / 2 }; };
            auto perpendicular = [](const auto& a, const auto& b) { return Vector{ a.y - b.y, b.x - a.x }; };
            auto euclideanDistance = [](const Vector& a, const Vector& b) -> Floating {
                auto dx = b.x - a.x, dy = b.y - a.y;
                return std::sqrt(dx * dx + dy * dy);
            };

            auto activateCircleEvent = [&](auto leftNode, auto rightNode) {
                const auto left = leftNode->leftSite;
                const auto middle = rightNode->leftSite;
                const auto right = rightNode->rightSite;
                Vector directionLeft = perpendicular(left->position, middle->position);
                Vector directionRight = perpendicular(middle->position, right->position);

                // Only add a new circle event if the two bisector edges will intersect in the future. This is not the case if they are parallel
                auto determinant = directionLeft.x * directionRight.y - directionLeft.y * directionRight.x;
                if (determinant > 0)
                {
                    Vector originLeft = center(left->position, middle->position);
                    Vector originRight = center(middle->position, right->position);

                    auto t = (originLeft.y * directionRight.x - directionRight.y * originLeft.x + 
                        directionRight.y * originRight.x - originRight.y * directionRight.x) / determinant;

                    Vector intersection = { originLeft.x + directionLeft.x * t, originLeft.y + directionLeft.y * t };
                    auto radius = euclideanDistance(intersection, middle->position);
                    rightNode->circleEvent = &circleEvents.emplace(rightNode, intersection, radius, intersection.y + radius, circleEventCounter++);
                }
            };

            auto deactivateCircleEvent = [&](auto beachlineNode) {
                if (beachlineNode->circleEvent)
                {
                    beachlineNode->circleEvent->active = false;
                    beachlineNode->circleEvent = nullptr;
                }
            };

            // Initialize beachline
            for (auto collinear = std::next(site); collinear != sites.end() && collinear->position.y == site->position.y; ++collinear)
            {
                if (collinear->position.x == site->position.x) continue;
                auto leftSite = site;
                addCollinearArc(leftSite, ++site, beachline.end());
            }
            if (beachline.empty())
            {
                auto leftSite = site;
                addArc(leftSite, leftSite, ++site, beachline.end());
            }
            ++site;

            while (site != sites.end() || !circleEvents.queue.empty())
            {
                // Always prioritize site events as they could deactivate a circle event that would otherwise has been processed
                if (site != sites.end() && (circleEvents.queue.empty() || site->position.y <= circleEvents.queue.front()->eventPosition))
                {
                    if (!site->cell)
                    {
                        ++site;
                        continue;
                    }

                    Bisector key = { site, site };
                    auto rightNode = beachline.lower_bound(key);

                    if (rightNode == beachline.end())
                    {
                        auto leftNode = std::prev(rightNode);
                        auto inserted = addArc(leftNode->rightSite, leftNode->rightSite, site, rightNode);
                        activateCircleEvent(leftNode, inserted);
                    }
                    else if (rightNode == beachline.begin())
                    {
                        auto leftNode = addArc(rightNode->leftSite, rightNode->leftSite, site, rightNode);
                        activateCircleEvent(++leftNode, rightNode);
                    }
                    else
                    {
                        deactivateCircleEvent(rightNode);
                        auto leftNode = std::prev(rightNode);
                        auto inserted = addArc(leftNode->rightSite, rightNode->leftSite, site, rightNode);

                        // Check and eventually activate circular event with the left and the right neighbour
                        activateCircleEvent(leftNode, inserted);
                        activateCircleEvent(++inserted, rightNode);
                    }
                    ++site;
                }
                else
                {
                    const auto& circleEvent = circleEvents.pop();
                    auto rightNode = circleEvent.node;
                    auto leftNode = std::prev(rightNode);

                    const auto left = leftNode->leftSite;
                    const auto middle = rightNode->leftSite;
                    const auto right = rightNode->rightSite;

                    auto& newEdge = edges.emplace_back(left->cell);
                    auto& newTwin = edges.emplace_back(right->cell);
                    newEdge.twin = &newTwin;
                    newTwin.twin = &newEdge;

                    Vertex vertex = { circleEvent.position, circleEvent.radius, { left->iterator, middle->iterator, right->iterator } };
                    if (!ignorePredicate(vertex))
                    {
                        vertex.incidentEdge = rightNode->edge;
                        vertices.push_back(vertex);
                        leftNode->edge->vertex = rightNode->edge->vertex = newTwin.vertex = &vertices.back();
                    }

                    auto interlink = [](Halfedge* a, Halfedge* b) { a->next = b, b->prev = a; };
                    interlink(&newEdge, leftNode->edge);
                    interlink(leftNode->edge->twin, rightNode->edge);
                    interlink(rightNode->edge->twin, &newTwin);

                    // We are allowed to modify the set key here as we know that the sorting order will not change. AB CD -> AD
                    leftNode->rightSite = rightNode->rightSite;
                    leftNode->edge = &newEdge;
                    beachline.erase(rightNode);

                    if (rightNode = leftNode; leftNode-- != beachline.begin())
                    {
                        deactivateCircleEvent(rightNode);
                        activateCircleEvent(leftNode, rightNode);
                    }
                    if (leftNode = rightNode; ++rightNode != beachline.end())
                    {
                        deactivateCircleEvent(rightNode);
                        activateCircleEvent(leftNode, rightNode);
                    }
                }

                // Skip and remove all incoming deactivated circle events
                while (!circleEvents.queue.empty() && !circleEvents.queue.front()->active)
                    circleEvents.pop();
            }

            // If a cell is infinite, link both infinite edges with each other so that each half edge has a successor and predecessor
            for (const auto& cell : cells)
            {
                auto start = cell.incidentEdge;
                auto leftEdge = start;
                while (leftEdge->prev != start && leftEdge->prev) leftEdge = leftEdge->prev;

                // Skip this cell if it is already enclosed
                if (leftEdge->prev) continue;

                auto rightEdge = start;
                while (rightEdge->next)	rightEdge = rightEdge->next;

                leftEdge->prev = rightEdge;
                rightEdge->next = leftEdge;
            }
        }
    };

    template <class Vector, class Iterator> auto generate(Iterator pointsBegin, Iterator pointsEnd)
    {
        return Diagram<Vector, Iterator>(pointsBegin, pointsEnd, defaultIgnorePredicate);
    }

    template <class Iterator> auto generate(Iterator pointsBegin, Iterator pointsEnd)
    {
        using Vector = typename std::iterator_traits<Iterator>::value_type;
        return Diagram<Vector, Iterator>(pointsBegin, pointsEnd, defaultIgnorePredicate);
    }

    template <class Vector, class Iterator, class IgnorePredicate>
    auto generate(Iterator pointsBegin, Iterator pointsEnd, IgnorePredicate&& ignorePredicate)
    {
        return Diagram<Vector, Iterator>(pointsBegin, pointsEnd, std::forward<IgnorePredicate>(ignorePredicate));
    }

    template <class Iterator, class IgnorePredicate>
    auto generate(Iterator pointsBegin, Iterator pointsEnd, IgnorePredicate&& ignorePredicate)
    {
        using Vector = typename std::iterator_traits<Iterator>::value_type;
        return Diagram<Vector, Iterator>(pointsBegin, pointsEnd, std::forward<IgnorePredicate>(ignorePredicate));
    }
};