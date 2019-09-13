#ifndef GRAPH_DIRECTED_ADJACENCY_LIST_HPP_9122019
#define GRAPH_DIRECTED_ADJACENCY_LIST_HPP_9122019

#include <cstddef>
#include <unordered_map>
#include <vector>
#include <iterator>
#include <utility>
#include <boost/iterator/iterator_facade.hpp>

#include "utils.hpp"
#include "edge.hpp"

namespace graph
{
    template <typename VertexType, typename EdgeType>
    class DirectedAdjacencyList final
    {
        private:
            std::unordered_map<VertexType, std::vector<EdgeType>> m_adjMap;
            std::size_t m_order = 0;

            std::vector<EdgeType>& getEdgeContainer(const VertexType& vertex)
            {
                auto slot(m_adjMap.find(vertex));
                if (slot != m_adjMap.end())
                    return slot->second;
                else
                    return m_adjMap.emplace(vertex, std::vector<EdgeType>()).first->second;
            }

        public:
            using vertex_t = VertexType;
            using edge_t = EdgeType;

            class ConstVertexIterator;
            class ConstEdgeIterator;

            DirectedAdjacencyList() = default;
            explicit DirectedAdjacencyList(std::size_t initialCapacity): m_adjMap(initialCapacity) {}
            DirectedAdjacencyList(const DirectedAdjacencyList<VertexType, EdgeType>& other) = default;

            DirectedAdjacencyList(DirectedAdjacencyList<VertexType, EdgeType>&& other)
            {
                if (&other != this)
                {
                    m_adjMap = std::move(other.m_adjMap);
                    m_order = other.m_order;
                }
            }

            std::size_t size() const {return m_adjMap.size();}
            std::size_t order() const {return m_order;}

            std::size_t indegree(const VertexType& vertex) const
            {
                std::size_t total = 0;
                for (const auto& elem: m_adjMap)
                {
                    for (const auto& edge: elem.second)
                    {
                        if (edge.head() == vertex)
                            ++total;
                    }
                }
                return total;
            }

            std::size_t outdegree(const VertexType& vertex) const {return m_adjMap.at(vertex).size();}

            ConstVertexIterator cbeginVertices() const {return ConstVertexIterator(m_adjMap.cbegin());}
            ConstVertexIterator cendVertices() const {return ConstVertexIterator(m_adjMap.cend());}

            ConstEdgeIterator cbeginEdges() const
            {
                auto begin(m_adjMap.cbegin());
                if (begin != m_adjMap.cend())
                    return ConstEdgeIterator(this, begin, begin->second.cbegin());
                else
                    return ConstEdgeIterator();
            }

            ConstEdgeIterator cendEdges() const {return ConstEdgeIterator();}

            void addVertex(const VertexType& vertex) {addVertexIfNotPresent(vertex);}
            bool addVertexIfNotPresent(const VertexType& vertex) {return !getEdgeContainer(vertex).size();}

            void addEdge(const EdgeType& edge)
            {
                addVertexIfNotPresent(edge.head());
                getEdgeContainer(edge.tail()).push_back(edge);
                ++m_order;
            }

            bool addEdgeIfNotPresent(const EdgeType& edge)
            {
                auto& incident = getEdgeContainer(edge.tail());
                for (const auto& existingEdge: incident)
                {
                    if (areParallel(edge, existingEdge))
                        return false;
                }
                addVertexIfNotPresent(edge.head());
                incident.push_back(edge);
                ++m_order;
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

            DirectedAdjacencyList<VertexType, EdgeType>& operator=(DirectedAdjacencyList<VertexType, EdgeType>&& other)
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
    class DirectedAdjacencyList<VertexType, EdgeType>::ConstVertexIterator final: public boost::iterator_facade<
        DirectedAdjacencyList<VertexType, EdgeType>::ConstVertexIterator, const VertexType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            using mapIter_t = typename std::unordered_map<VertexType, std::vector<EdgeType>>::const_iterator;

            mapIter_t m_mapIter;

            const VertexType& dereference() const {return m_mapIter->first;}
            bool equal(const ConstVertexIterator& other) const {return m_mapIter == other.m_mapIter;}
            void increment() {++m_mapIter;}

        public:
            ConstVertexIterator() = default;
            explicit ConstVertexIterator(const mapIter_t& mapIter): m_mapIter(mapIter) {}
    };

    template <typename VertexType, typename EdgeType>
    class DirectedAdjacencyList<VertexType, EdgeType>::ConstEdgeIterator final: public boost::iterator_facade<
        DirectedAdjacencyList<VertexType, EdgeType>::ConstEdgeIterator, const EdgeType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            using mapIter_t = typename std::unordered_map<VertexType, std::vector<EdgeType>>::const_iterator;
            using vecIter_t = typename std::vector<EdgeType>::const_iterator;

            const DirectedAdjacencyList<VertexType, EdgeType>* m_outer = nullptr;
            mapIter_t m_mapIter;
            vecIter_t m_vecIter;

            bool isPastTheEnd() const
            {
                return !m_outer || m_mapIter == m_outer->m_adjMap.cend() ||
                       (std::next(m_mapIter) == m_outer->m_adjMap.cend() && m_vecIter == m_mapIter->second.cend());
            }

            void seekNextNotIsolated()
            {
                for (; m_mapIter != m_outer->m_adjMap.cend(); ++m_mapIter)
                {
                    if (m_mapIter->second.size())
                    {
                        m_vecIter = m_mapIter->second.cbegin();
                        break;
                    }
                }
            }

            const EdgeType& dereference() const {return *m_vecIter;}

            bool equal(const ConstEdgeIterator& other) const
            {
                bool otherPastTheEnd = other.isPastTheEnd();
                if (isPastTheEnd())
                    return otherPastTheEnd;
                else if (otherPastTheEnd)
                    return false;
                else
                    return m_mapIter == other.m_mapIter && m_vecIter == other.m_vecIter;
            }

            void increment()
            {
                if (m_vecIter + 1 == m_mapIter->second.cend())
                {
                    ++m_mapIter;
                    seekNextNotIsolated();
                }
                else
                    ++m_vecIter;
            }

        public:
            ConstEdgeIterator() = default;
            ConstEdgeIterator(
                const DirectedAdjacencyList<VertexType, EdgeType>* outer,
                const mapIter_t& mapIter,
                const vecIter_t& vecIter):
                    m_outer(outer), m_mapIter(mapIter), m_vecIter(vecIter)
                {
                    if (outer)
                        seekNextNotIsolated();
                }
    };

    template <typename VertexType>
    using basicDirectedAdjacencyList_t = DirectedAdjacencyList<VertexType, Edge<VertexType>>;

    template <typename VertexType, typename WeightType>
    using weightedDirectedAdjacencyList_t = DirectedAdjacencyList<VertexType, WeightedEdge<VertexType, WeightType>>;
}

#endif
