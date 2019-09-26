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
            DirectedAdjacencyList(const DirectedAdjacencyList<VertexType, EdgeType>& other) = default;
            DirectedAdjacencyList(DirectedAdjacencyList<VertexType, EdgeType>&& other) noexcept:
                m_adjMap(std::move(other.m_adjMap)), m_order(other.m_order) {}

            std::size_t size() const {return m_adjMap.size();}
            std::size_t order() const {return m_order;}

            std::size_t indegree(const VertexType& vertex) const {return m_adjMap.at(vertex).indegree;}
            std::size_t outdegree(const VertexType& vertex) const {return m_adjMap.at(vertex).outdegree;}

            auto vertices() const
            {
                VertexType (*transform)(const typename decltype(m_adjMap)::value_type&) =
                    std::get<const VertexType, VertexInfo>;
                return std::make_pair(
                    boost::make_transform_iterator(m_adjMap.cbegin(), transform),
                    boost::make_transform_iterator(m_adjMap.cend(), transform));
            }

            auto edges() const
            {
                auto begin(m_adjMap.cbegin());
                EdgeIterator rangeStart(begin != m_adjMap.cend()
                    ? EdgeIterator(this, begin, begin->second.outEdges.cbegin())
                    : EdgeIterator());
                return std::make_pair(rangeStart, EdgeIterator());
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

            DirectedAdjacencyList<VertexType, EdgeType>& operator=(
                const DirectedAdjacencyList<VertexType, EdgeType>& other) = default;

            DirectedAdjacencyList<VertexType, EdgeType>& operator=(
                DirectedAdjacencyList<VertexType, EdgeType>&& other) noexcept
            {
                if (&other != this)
                {
                    m_adjMap = std::move(other.m_adjMap);
                    m_order = other.m_order;
                }
                return *this;
            }
    };

    template <typename VertexType, typename EdgeType>
    class DirectedAdjacencyList<VertexType, EdgeType>::EdgeIterator final: public boost::iterator_facade<
        DirectedAdjacencyList<VertexType, EdgeType>::EdgeIterator, const EdgeType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            using mapIter_t = typename std::unordered_map<VertexType, VertexInfo>::const_iterator;
            using listIter_t = typename std::forward_list<EdgeType>::const_iterator;

            const DirectedAdjacencyList<VertexType, EdgeType>* m_outer = nullptr;
            mapIter_t m_mapIter;
            listIter_t m_listIter;

            bool isPastTheEnd() const
            {
                return !m_outer || m_mapIter == m_outer->m_adjMap.cend() ||
                       (std::next(m_mapIter) == m_outer->m_adjMap.cend() &&
                        m_listIter == m_mapIter->second.outEdges.cend());
            }

            void seekNextNotDeadEnd()
            {
                for (; m_mapIter != m_outer->m_adjMap.cend(); ++m_mapIter)
                {
                    if (!m_mapIter->second.outEdges.empty())
                    {
                        m_listIter = m_mapIter->second.outEdges.cbegin();
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
                    return m_mapIter == other.m_mapIter && m_listIter == other.m_listIter;
            }

            void increment()
            {
                if (std::next(m_listIter) == m_mapIter->second.outEdges.cend())
                {
                    ++m_mapIter;
                    seekNextNotDeadEnd();
                }
                else
                    ++m_listIter;
            }

        public:
            EdgeIterator() = default;
            EdgeIterator(
                const DirectedAdjacencyList<VertexType, EdgeType>* outer,
                const mapIter_t& mapIter,
                const listIter_t& listIter):
                    m_outer(outer), m_mapIter(mapIter), m_listIter(listIter)
                {
                    if (outer)
                        seekNextNotDeadEnd();
                }
    };

    template <typename VertexType>
    using basicDirectedAdjacencyList_t = DirectedAdjacencyList<VertexType, Edge<VertexType>>;

    template <typename VertexType, typename WeightType>
    using weightedDirectedAdjacencyList_t = DirectedAdjacencyList<VertexType, WeightedEdge<VertexType, WeightType>>;
}

#endif
