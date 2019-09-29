#ifndef GRAPH_DIRECTED_ADJACENCY_LIST_HPP_9122019
#define GRAPH_DIRECTED_ADJACENCY_LIST_HPP_9122019

#include <cstddef>
#include <unordered_map>
#include <forward_list>
#include <iterator>
#include <utility>
#include <unordered_set>
#include <boost/make_shared.hpp>
#include <boost/shared_container_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include "utils.hpp"
#include "edge.hpp"

namespace graph
{
    template <typename VertexType, typename EdgeType>
    class DirectedAdjacencyList final
    {
        private:
            struct VertexInfo
            {
                std::size_t indegree = 0, outdegree = 0;
                std::forward_list<EdgeType> outEdges;
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

            DirectedAdjacencyList() = default;
            explicit DirectedAdjacencyList(std::size_t initialCapacity): m_adjMap(initialCapacity) {}

            std::size_t size() const {return m_adjMap.size();}
            std::size_t order() const {return m_order;}

            std::size_t indegree(const VertexType& vertex) const {return m_adjMap.at(vertex).indegree;}
            std::size_t outdegree(const VertexType& vertex) const {return m_adjMap.at(vertex).outdegree;}

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

            auto inNeighbors(const VertexType& vertex) const
            {
                auto neighbors(boost::make_shared<std::unordered_set<VertexType>>());
                for (const auto& elem: m_adjMap)
                {
                    for (const auto& edge: elem.second.outEdges)
                    {
                        if (edge.head() == vertex)
                            neighbors->insert(edge.tail());
                    }
                }
                return boost::make_shared_container_range(neighbors);
            }

            auto outNeighbors(const VertexType& vertex) const
            {
                auto neighbors(boost::make_shared<std::unordered_set<VertexType>>());
                for (const auto& edge: m_adjMap.at(vertex).outEdges)
                    neighbors->insert(edge.head());
                return boost::make_shared_container_range(neighbors);
            }

            auto incidentInEdges(const VertexType& vertex)
            {
                auto incident(boost::make_shared<std::unordered_set<EdgeType>>());
                for (const auto& elem: m_adjMap)
                {
                    for (const auto& edge: elem.second.outEdges)
                    {
                        if (edge.head() == vertex)
                            incident->insert(edge);
                    }
                }
                return boost::make_shared_container_range(incident);
            }

            auto incidentOutEdges(const VertexType& vertex) const
            {
                const std::forward_list<EdgeType>& edges = m_adjMap.at(vertex).outEdges;
                return std::make_pair(edges.cbegin(), edges.cend());
            }

            void addVertex(const VertexType& vertex) {addVertexIfNotPresent(vertex);}
            bool addVertexIfNotPresent(const VertexType& vertex) {return getVertexInfo(vertex).outEdges.empty();}

            void addEdge(const EdgeType& edge)
            {
                VertexInfo &tailInfo = getVertexInfo(edge.tail()), &headInfo = getVertexInfo(edge.head());
                tailInfo.outEdges.push_front(edge);
                ++tailInfo.indegree, ++headInfo.outdegree, ++m_order;
            }

            bool addEdgeIfNotPresent(const EdgeType& edge)
            {
                VertexInfo& tailInfo = getVertexInfo(edge.tail());
                for (const auto& existingEdge: tailInfo.outEdges)
                {
                    if (areParallel(edge, existingEdge))
                        return false;
                }
                VertexInfo& headInfo = getVertexInfo(edge.head());
                tailInfo.outEdges.push_front(edge);
                ++tailInfo.indegree, ++headInfo.outDegree, ++m_order;
                return true;
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
    class DirectedAdjacencyList<VertexType, EdgeType>::EdgeIterator final: public boost::iterator_facade<
        DirectedAdjacencyList<VertexType, EdgeType>::EdgeIterator, const EdgeType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            using mapIter_t = typename decltype(m_adjMap)::const_iterator;

            std::pair<mapIter_t, mapIter_t> m_mapRange;
            typename std::forward_list<EdgeType>::const_iterator m_listIter;

            bool isPastTheEnd() const
            {
                return m_mapRange.first == m_mapRange.second ||
                       (std::next(m_mapRange.first) == m_mapRange.second &&
                        m_listIter == m_mapRange.first->second.outEdges.cend());
            }

            void seekNextNotDeadEnd()
            {
                for (; m_mapRange.first != m_mapRange.second; ++m_mapRange.first)
                {
                    if (!m_mapRange.first->second.outEdges.empty())
                    {
                        m_listIter = m_mapRange.first->second.outEdges.cbegin();
                        break;
                    }
                }
            }

            const EdgeType& dereference() const {return *m_listIter;}

            bool equal(const EdgeIterator& other) const
            {
                bool otherPastTheEnd = other.isPastTheEnd();
                if (isPastTheEnd())
                    return otherPastTheEnd;
                else if (otherPastTheEnd)
                    return false;
                else
                    return m_mapRange.first == other.m_mapRange.first && m_listIter == other.m_listIter;
            }

            void increment()
            {
                if (std::next(m_listIter) == m_mapRange.first->second.outEdges.cend())
                {
                    ++m_mapRange.first;
                    seekNextNotDeadEnd();
                }
                else
                    ++m_listIter;
            }

        public:
            EdgeIterator() = default;
            explicit EdgeIterator(const std::pair<mapIter_t, mapIter_t>& mapRange):
                    m_mapRange(mapRange), m_listIter(m_mapRange.first->second.outEdges.cbegin()) {seekNextNotDeadEnd();}
    };

    template <typename VertexType>
    using basicDirectedAdjacencyList_t = DirectedAdjacencyList<VertexType, Edge<VertexType, true>>;

    template <typename VertexType, typename WeightType>
    using weightedDirectedAdjacencyList_t =
        DirectedAdjacencyList<VertexType, WeightedEdge<VertexType, true, WeightType>>;
}

#endif
