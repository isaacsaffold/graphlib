#ifndef GRAPH_UNDIRECTED_ADJACENCY_LIST_HPP_9192019
#define GRAPH_UNDIRECTED_ADJACENCY_LIST_HPP_9192019

#include <cstddef>
#include <unordered_map>
#include <forward_list>
#include <utility>
#include <iterator>
#include <unordered_set>
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_container_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include "utils.hpp"
#include "edge.hpp"

namespace graph
{
    template <typename VertexType, typename EdgeType>
    class UndirectedAdjacencyList final
    {
        private:
            struct EdgeWrapper
            {
                EdgeType edge;
                bool marked;
            };

            struct VertexInfo
            {
                std::size_t degree = 0;
                std::forward_list<EdgeWrapper> incidentEdges;
            };

            class EdgeIterator;

            std::unordered_map<VertexType, VertexInfo> m_adjMap;
            std::size_t m_order = 0;

            VertexInfo& getVertexInfo(const VertexType& vertex)
            {
                auto slot(m_adjMap.find(vertex));
                if (slot != m_adjMap.end())
                    return slot->second;
                else
                    return m_adjMap.emplace(vertex, VertexInfo{}).first->second;
            }

        public:
            using vertex_t = VertexType;
            using edge_t = EdgeType;

            UndirectedAdjacencyList() = default;
            explicit UndirectedAdjacencyList(std::size_t initialCapacity): m_adjMap(initialCapacity) {}

            std::size_t size() const {return m_adjMap.size();}
            std::size_t order() const {return m_order;}

            std::size_t degree(const VertexType& vertex) const {return m_adjMap.at(vertex).degree;}

            auto vertices() const
            {
                const VertexType& (*transform)(const typename decltype(m_adjMap)::value_type&) =
                    std::get<const VertexType, VertexInfo>;
                return std::make_pair(
                    boost::make_transform_iterator(m_adjMap.cbegin(), transform),
                    boost::make_transform_iterator(m_adjMap.cend(), transform));
            }

            auto edges() const
            {
                auto range(std::make_pair(m_adjMap.cbegin(), m_adjMap.cend()));
                return std::make_pair(range.first != range.second ? EdgeIterator(range) : EdgeIterator(),
                    EdgeIterator());
            }

            auto neighbors(const VertexType& vertex) const
            {
                auto neighborSet(boost::make_shared<std::unordered_set<VertexType>>());
                for (const auto& wrapper: m_adjMap.at(vertex).incidentEdges)
                    neighborSet->insert(wrapper.edge.head());
                return boost::make_shared_container_range(neighborSet);
            }

            auto incidentEdges(const VertexType& vertex) const
            {
                const std::forward_list<EdgeWrapper>& edges = m_adjMap.at(vertex).incidentEdges;
                auto transform = [](const EdgeWrapper& wrapper){return wrapper.edge;};
                return std::make_pair(
                    boost::make_transform_iterator(edges.cbegin(), transform),
                    boost::make_transform_iterator(edges.cend(), transform));
            }

            void addVertex(const VertexType& vertex) {addVertexIfNotPresent(vertex);}
            bool addVertexIfNotPresent(const VertexType& vertex) {return getVertexInfo(vertex).incidentEdges.empty();}

            void addEdge(const EdgeType& edge)
            {
                VertexInfo& first = getVertexInfo(edge.tail());
                first.incidentEdges.push_front({edge, true});
                if (edge.tail() == edge.head())
                    first.degree += 2;
                else
                {
                    VertexInfo& second = getVertexInfo(edge.head());
                    second.incidentEdges.push_front({edge, false});
                    ++first.degree, ++second.degree;
                }
                ++m_order;
            }

            bool addEdgeIfNotPresent(const EdgeType& edge)
            {
                VertexInfo& first = getVertexInfo(edge.tail());
                if (!std::count(first.incidentEdges.begin(), first.incidentEdges.end(), edge))
                {
                    first.incidentEdges.push_front({edge, true});
                    if (edge.tail() == edge.head())
                        first.degree += 2;
                    else
                    {
                        VertexInfo& second = getVertexInfo(edge.head());
                        second.incidentEdges.push_front({edge, false});
                        ++first.degree, ++second.degree;
                    }
                    ++m_order;
                    return true;
                }
                else
                    return false;
            }

            void addVertex(VertexType&& vertex) {addVertex(static_cast<const VertexType&>(vertex));}

            bool addVertexIfNotPresent(VertexType&& vertex)
            {
                return addVertexIfNotPresent(static_cast<const VertexType&>(vertex));
            }

            void addEdge(EdgeType&& edge) {addEdge(static_cast<const EdgeType&>(edge));}
            bool addEdgeIfNotPresent(EdgeType&& edge) {return addEdgeIfNotPresent(static_cast<const EdgeType&>(edge));}

            template <typename... Args>
            void emplaceVertex(Args&&... args) {addVertex(VertexType(std::forward<Args>(args)...));}

            template <typename... Args>
            bool emplaceVertexIfNotPresent(Args&&... args)
            {
                return addVertexIfNotPresent(VertexType(std::forward<Args>(args)...));
            }

            template <typename... Args>
            void emplaceEdge(Args&&... args) {addEdge(EdgeType(std::forward<Args>(args)...));}

            template <typename... Args>
            bool emplaceEdgeIfNotPresent(Args&&... args)
            {
                return addEdgeIfNotPresent(EdgeType(std::forward<Args>(args)...));
            }
    };

    template <typename VertexType, typename EdgeType>
    class UndirectedAdjacencyList<VertexType, EdgeType>::EdgeIterator: public boost::iterator_facade<
        UndirectedAdjacencyList<VertexType, EdgeType>::EdgeIterator, const EdgeType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            using mapIter_t = typename decltype(m_adjMap)::const_iterator;

            static bool isMarked(const EdgeWrapper& wrapper) {return wrapper.marked;}

            std::pair<mapIter_t, mapIter_t> m_mapRange;
            typename boost::filter_iterator<
                bool (*)(const EdgeWrapper&), std::forward_list<EdgeWrapper>::const_iterator> m_edgeIter;

            bool isPastTheEnd() const
            {
                return m_mapRange.first == m_mapRange.second ||
                       (std::next(m_mapRange.first) == m_mapRange.second && m_edgeIter.base() == m_edgeIter.end();
            }

            const EdgeType& dereference() const {return m_edgeIter->edge;}

            bool equal(const EdgeIterator& other) const
            {
                bool otherPastTheEnd = other.isPastTheEnd();
                if (isPastTheEnd())
                    return otherPastTheEnd;
                else if (otherPastTheEnd)
                    return false;
                else
                    return m_mapIter == other.m_mapIter && m_edgeIter == other.m_edgeIter;
            }

            void increment()
            {
                auto next(std::next(m_edgeIter));
                if (next.base() != m_edgeIter.end())
                    m_edgeIter = next;
                else
                {
                    while (++m_mapRange.first != m_mapRange.second)
                    {
                        const auto& edges = m_mapRange.first->second.incidentEdges;
                        m_edgeIter = boost::make_filter_iterator(isMarked, edges.cbegin(), edges.cend());
                        if (m_edgeIter.base() != m_edgeIter.end())
                            break;
                    }
                }
            }

        public:
            EdgeIterator() = default;

            explicit EdgeIterator(const std::pair<mapIter_t, mapIter_t>& mapRange): m_mapRange(mapRange)
            {
                const auto& edges = m_mapRange.first->second.incidentEdges;
                m_edgeIter = boost::make_filter_iterator(isMarked, edges.cbegin(), edges.cend());
                if (m_edgeIter.base() == m_edgeIter.end())
                    increment();
            }
    };

    template <typename VertexType>
    using basicUndirectedAdjacencyList_t = UndirectedAdjacencyList<VertexType, Edge<VertexType, false>>;

    template <typename VertexType, typename WeightType>
    using weightedUndirectedAdjacencyList_t =
        UndirectedAdjacencyList<VertexType, WeightedEdge<VertexType, false, WeightType>>;
}

#endif
