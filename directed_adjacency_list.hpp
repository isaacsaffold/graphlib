#ifndef GRAPH_DIRECTED_ADJACENCY_LIST_HPP_1092019
#define GRAPH_DIRECTED_ADJACENCY_LIST_HPP_1092019

#include <cstddef>
#include <unordered_map>
#include <forward_list>
#include <type_traits>
#include <utility>
#include <iterator>
#include <boost/utility/result_of.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include "constraints.hpp"
#include "edge_wrapper.hpp"

namespace graph
{
    template <typename Vertex, typename Edge = void, Constraints constraints = Constraints::SIMPLY_CONNECTED,
        typename SizeType = std::size_t>
    class DirectedAdjacencyList final
    {
        private:
            template <typename Value>
            class EdgeIterator;

            struct ImplicitAdjacencyInfo
            {
                Vertex vertex;
            };

            struct AdjacencyInfo
            {
                Vertex vertex;
                Edge edge;
            };

            using adjInfo_t = std::conditional_t<std::is_void<Edge>::value, ImplicitAdjacencyInfo, AdjacencyInfo>;

            struct VertexInfo
            {
                std::forward_list<adjInfo_t> outNeighbors;
                SizeType indegree = 0, outdegree = 0;
            };

            template <typename E = Edge, std::enable_if_t<std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<Vertex, E> edgeWrapper(adjInfo_t& adjInfo, const Vertex& head)
            {
                return {adjInfo.vertex, head};
            }

            template <typename E = Edge, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<Vertex, E> edgeWrapper(adjInfo_t& adjInfo, const Vertex& head)
            {
                return {adjInfo.vertex, head, adjInfo.edge};
            }

            static const edgeWrapper_t<Vertex, Edge> edgeWrapper(const adjInfo_t& adjInfo, const Vertex& head)
            {
                return edgeWrapper(const_cast<adjInfo_t&>(adjInfo), head);
            }

            static auto edgeWrapperFrom(std::pair<const Vertex, VertexInfo>& elem)
            {
                return [&](adjInfo_t& adjInfo){return edgeWrapper(adjInfo, elem.first);};
            }

            static auto constEdgeWrapperFrom(const std::pair<const Vertex, VertexInfo>& elem)
            {
                return [&](const adjInfo_t& adjInfo) -> const edgeWrapper_t<Vertex, Edge>
                {
                    return edgeWrapper(adjInfo, elem.first);
                };
            }

            using outEdgeIter_t = boost::transform_iterator<
                typename boost::result_of<decltype(
                    &DirectedAdjacencyList::edgeWrapperFrom)(std::pair<const Vertex, VertexInfo>&)>::type,
                typename std::forward_list<adjInfo_t>::iterator>;
            using constOutEdgeIter_t = boost::transform_iterator<
                typename boost::result_of<decltype(
                    &DirectedAdjacencyList::constEdgeWrapperFrom)(const std::pair<const Vertex, VertexInfo>&)>::type,
                typename std::forward_list<adjInfo_t>::const_iterator>;

            static std::pair<outEdgeIter_t, outEdgeIter_t> outEdgeWrappers(std::pair<const Vertex, VertexInfo>& elem)
            {
                auto transform(edgeWrapperFrom(elem));
                return std::make_pair(
                    boost::make_transform_iterator(elem.second.outNeighbors.begin(), transform),
                    boost::make_transform_iterator(elem.second.outNeighbors.end(), transform));
            }

            static std::pair<constOutEdgeIter_t, constOutEdgeIter_t> outEdgeWrappers(
                const std::pair<const Vertex, VertexInfo>& elem)
            {
                auto transform(constEdgeWrapperFrom(elem));
                return std::make_pair(
                    boost::make_transform_iterator(elem.second.outNeighbors.begin(), transform),
                    boost::make_transform_iterator(elem.second.outNeighbors.end(), transform));
            }

            static auto makeAreAdjacent(const Vertex& tail, const Vertex& head)
            {
                return [&](const edgeWrapper_t<Vertex, Edge>& wrapper)
                {
                    return tail == wrapper.tail && head == wrapper.head;
                };
            }

            static auto makeIsInNeighbor(const Vertex& head)
            {
                return [&](const edgeWrapper_t<Vertex, Edge>& wrapper){return head == wrapper.head;};
            }

            std::unordered_map<Vertex, VertexInfo> m_adjMap;
            SizeType m_order = 0;

        public:
            using vertex_t = Vertex;
            using edge_t = Edge;
            using customSize_t = SizeType;
            using constVertexIter_t = boost::transform_iterator<
                const Vertex& (*)(const std::pair<const Vertex, VertexInfo>&),
                typename decltype(m_adjMap)::const_iterator>;
            using edgeIter_t = EdgeIterator<edgeWrapper_t<Vertex, Edge>>;
            using constEdgeIter_t = EdgeIterator<const edgeWrapper_t<Vertex, Edge>>;
            using incidentEdgeIter_t = boost::filter_iterator<
                decltype(makeAreAdjacent(std::declval<Vertex>(), std::declval<Vertex>())),
                typename std::forward_list<adjInfo_t>::iterator>;
            using constIncidentEdgeIter_t = boost::filter_iterator<
                decltype(makeAreAdjacent(std::declval<Vertex>(), std::declval<Vertex>())),
                typename std::forward_list<adjInfo_t>::const_iterator>;
            using inNeighborIter_t = boost::filter_iterator<
                decltype(makeIsInNeighbor(std::declval<Vertex>())), edgeIter_t>;
            using constInNeighborIter_t = boost::filter_iterator<
                decltype(makeIsInNeighbor(std::declval<Vertex>())), constEdgeIter_t>;
            using outNeighborIter_t = outEdgeIter_t;
            using constOutNeighborIter_t = constOutEdgeIter_t;

            static constexpr Constraints getConstraints() {return constraints;}
            static constexpr bool isDirected() {return true;}

            DirectedAdjacencyList() = default;
            explicit DirectedAdjacencyList(SizeType initialCapacity): m_adjMap(initialCapacity) {}

        private:
            template <typename IncidentEdgeIter, typename MapIter>
            std::pair<IncidentEdgeIter, IncidentEdgeIter> implIncidentEdges(const Vertex& tail, const Vertex& head)
            {
                MapIter mapIter(m_adjMap.find(tail));
                if (mapIter != m_adjMap.end())
                {
                    auto predicate(makeAreAdjacent(tail, head));
                    auto range(outEdgeWrappers(*mapIter));
                    return std::make_pair(
                        boost::make_filter_iterator(predicate, range.first, range.second),
                        boost::make_filter_iterator(range.second, range.second));
                }
                else
                    return std::pair<IncidentEdgeIter, IncidentEdgeIter>();
            }

            template <typename InNeighborIter, typename EdgeIter>
            std::pair<InNeighborIter, InNeighborIter> implInNeighbors(const Vertex& head)
            {
                if (!m_adjMap.count(head))
                    return std::pair<InNeighborIter, InNeighborIter>();
                else
                {
                    auto predicate(makeIsInNeighbor(head));
                    EdgeIter range(edges());
                    return std::make_pair(
                        boost::make_filter_iterator(predicate, range.first, range.second),
                        boost::make_filter_iterator(range.second, range.second));
                }
            }

            template <typename V>
            edgeWrapper_t<Vertex, Edge> getAdjInfoAndUpdate(V&& tail, V&& head)
            {
                auto tailIter(m_adjMap.find(tail));
                if (tailIter == m_adjMap.end())
                    tailIter = m_adjMap.emplace(tail, VertexInfo{}).first;
                auto headIter(m_adjMap.find(head));
                if (headIter == m_adjMap.end())
                    headIter = m_adjMap.emplace(head, VertexInfo{}).first;
                VertexInfo& tailInfo = tailIter->second;
                ++m_order, ++tailInfo.outdegree, ++headIter->second.indegree;
                adjInfo_t& adjInfo = *tailInfo.outNeighbors.insert_after(tailInfo.outNeighbors.before_begin(), {head});
                return edgeWrapper(adjInfo, head);
            }

            template <typename V, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) == 0 && std::is_same<V, V>::value, bool> = true>
            edgeWrapper_t<typename std::remove_reference_t<V>, Edge> implAddEdge(V&& tail, V&& head)
            {
                return getAdjInfoAndUpdate(tail, head);
            }

            template <typename V, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) != 0 && std::is_same<V, V>::value, bool> = true>
            edgeWrapper_t<typename std::remove_reference_t<V>, Edge> implAddEdge(V&& tail, V&& head)
            {
                if (tail == head)
                    throw ConstraintViolationException(Constraints::NO_LOOPS);
                return getAdjInfoAndUpdate(tail, head);
            }

        public:
            SizeType size() const {return m_adjMap.size();}
            SizeType order() const {return m_order;}

            SizeType indegree(const Vertex& vertex) const {return m_adjMap.at(vertex).indegree;}
            SizeType outdegree(const Vertex& vertex) const {return m_adjMap.at(vertex).outdegree;}

            bool contains(const Vertex& vertex) const {return m_adjMap.count(vertex);}

            template <typename E = Edge, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            bool contains(const E& edge) const
            {
                for (const auto& elem: m_adjMap)
                {
                    for (const adjInfo_t& adjInfo: elem.second.outNeighbors)
                    {
                        if (edge == adjInfo.edge)
                            return true;
                    }
                }
                return false;
            }

            // if `tail` or `head` are not contained in the graph, returns `false`
            bool areAdjacent(const Vertex& tail, const Vertex& head) const
            {
                auto iter(m_adjMap.find(tail));
                if (iter == m_adjMap.end)
                    return false;
                for (const adjInfo_t& adjInfo: iter->second.outNeighbors)
                {
                    if (head == adjInfo.vertex)
                        return true;
                }
                return false;
            }

            // TODO: Implement interoperability between const and non-const iterators if not built-in.

            std::pair<constVertexIter_t, constVertexIter_t> vertices() const
            {
                const Vertex& (*transform)(const std::pair<const Vertex, VertexInfo>&)
                    = std::get<const Vertex, VertexInfo>;
                return std::make_pair(
                    boost::make_transform_iterator(m_adjMap.cbegin(), transform),
                    boost::make_transform_iterator(m_adjMap.cend(), transform));
            }

            std::pair<constVertexIter_t, constVertexIter_t> constVertices() const {return vertices();}

            template <typename E = Edge, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            std::pair<edgeIter_t, edgeIter_t> edges()
            {
                auto range(std::make_pair(m_adjMap.begin(), m_adjMap.end()));
                return std::make_pair(range.first != range.second ? edgeIter_t(range) : edgeIter_t(),
                    edgeIter_t());
            }

            std::pair<constEdgeIter_t, constEdgeIter_t> edges() const
            {
                auto range(std::make_pair(m_adjMap.cbegin(), m_adjMap.cend()));
                return std::make_pair(range.first != range.second ? constEdgeIter_t(range) : constEdgeIter_t(),
                    constEdgeIter_t());
            }

            std::pair<constEdgeIter_t, constEdgeIter_t> constEdges() const {return edges();}

            template <typename E = Edge, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            std::pair<incidentEdgeIter_t, incidentEdgeIter_t> incidentEdges(const Vertex& tail, const Vertex& head)
            {
                return implIncidentEdges<incidentEdgeIter_t, typename decltype(m_adjMap)::iterator>(tail, head);
            }

            std::pair<constIncidentEdgeIter_t, constIncidentEdgeIter_t> incidentEdges(const Vertex& tail,
                const Vertex& head) const
            {
                return const_cast<DirectedAdjacencyList*>(this)->implIncidentEdges<
                    constIncidentEdgeIter_t, typename decltype(m_adjMap)::const_iterator>(tail, head);
            }

            std::pair<constIncidentEdgeIter_t, constIncidentEdgeIter_t> constIncidentEdges(const Vertex& tail,
                const Vertex& head) const
            {
                return incidentEdges(tail, head);
            }

            template <typename E = Edge, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            std::pair<inNeighborIter_t, inNeighborIter_t> inNeighbors(const Vertex& head)
            {
                return implInNeighbors<inNeighborIter_t, edgeIter_t>(head);
            }

            std::pair<constInNeighborIter_t, constInNeighborIter_t> inNeighbors(const Vertex& head) const
            {
                return const_cast<DirectedAdjacencyList*>(this)->
                    implInNeighbors<constInNeighborIter_t, constEdgeIter_t>(head);
            }

            std::pair<constInNeighborIter_t, constInNeighborIter_t> constInNeighbors(const Vertex& head) const
            {
                return inNeighbors(head);
            }

            template <typename E = Edge, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            std::pair<outNeighborIter_t, outNeighborIter_t> outNeighbors(const Vertex& tail)
            {
                auto mapIter(m_adjMap.find(tail));
                return mapIter != m_adjMap.end()
                    ? outEdgeWrappers(*mapIter)
                    : std::pair<outNeighborIter_t, outNeighborIter_t>();
            }

            std::pair<constOutNeighborIter_t, constOutNeighborIter_t> outNeighbors(const Vertex& tail) const
            {
                auto mapIter(m_adjMap.find(tail));
                return mapIter != m_adjMap.end()
                    ? outEdgeWrappers(*mapIter)
                    : std::pair<constOutNeighborIter_t, constOutNeighborIter_t>();
            }

            std::pair<constOutNeighborIter_t, constOutNeighborIter_t> constOutNeighbors(const Vertex& tail) const
            {
                return outNeighbors(tail);
            }

            template <typename V, std::enable_if_t<
                (constraints & Constraints::CONNECTED) == 0 && std::is_same<V, V>::value, bool> = true>
            bool addVertex(V&& vertex)
            {
                if (m_adjMap.count(vertex))
                    return false;
                else
                    return m_adjMap.emplace(vertex, VertexInfo{}).second;
            }

            template <typename V>
            edgeWrapper_t<typename std::remove_reference_t<V>, Edge> addEdge(V&& tail, V&& head)
            {
                return implAddEdge(tail, head);
            }

            template <typename V, typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            edgeWrapper_t<typename std::remove_reference_t<V>, typename std::remove_reference_t<E>>
                addEdge(V&& tail, V&& head, E&& edge)
            {
                auto wrapper(implAddEdge(tail, head));
                wrapper.edge = std::forward<E>(edge);
                return wrapper;
            }

            // TODO: Add more mutator methods (e.g. for removing vertices and edges).
    };

    /*
     * Almost conforms to `LegacyForwardIterator`. There is only one deviation and it is insignificant in most contexts.
     * For two instances `a` and `b`, it is possible that `a == b && &*a != &*b`. However, for each pair of members
     * `a->m` and `b->m`, if `a == b` then `&(a->m) == &(b->m)`.
     *
     * `Value` is always either `edgeWrapper_t<Vertex, Edge>` or `const edgeWrapper_t<Vertex, Edge>`.
     */
    template <typename Vertex, typename Edge, Constraints constraints, typename SizeType>
    template <typename Value>
    class DirectedAdjacencyList<Vertex, Edge, constraints, SizeType>::EdgeIterator final: public boost::iterator_facade<
        DirectedAdjacencyList<Vertex, Edge, constraints, SizeType>::EdgeIterator<Value>, Value, std::input_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            using mapIter_t = std::conditional_t<std::is_const<Value>::value,
                typename decltype(m_adjMap)::const_iterator, typename decltype(m_adjMap)::iterator>;

            std::pair<mapIter_t, mapIter_t> m_mapRange;
            std::conditional_t<std::is_const<Value>::value,
                typename std::forward_list<adjInfo_t>::const_iterator, typename std::forward_list<adjInfo_t>::iterator>
                m_listIter;
            // is not of type `Value`, as it is mutated even when `Value` is const
            edgeWrapper_t<Vertex, Edge> m_current;

            std::forward_list<edgeWrapper_t<Vertex, Edge>>& getOutNeighbors() const
            {
                return m_mapRange.first->second.outNeighbors;
            }

            bool isPastTheEnd() const
            {
                return m_mapRange.first == m_mapRange.second ||
                       (std::next(m_mapRange.first) == m_mapRange.second &&
                        m_listIter == const_cast<const std::forward_list<edgeWrapper_t<Vertex, Edge>>&>(
                            getOutNeighbors()).end());
            }

            void seekNextNotDeadEnd()
            {
                for (; m_mapRange.first != m_mapRange.second; ++m_mapRange.first)
                {
                    auto& outNeighbors = getOutNeighbors();
                    if (!outNeighbors.empty())
                    {
                        m_listIter = outNeighbors.begin();
                        m_current = edgeWrapper(*m_listIter, m_mapRange.first->first);
                        break;
                    }
                }
            }

            Value& dereference() const {return m_current;}

            // TODO: Ensure that this works for a mixture of const and non-const `Value`.
            template <typename OtherValue>
            bool equal(const EdgeIterator<OtherValue>& other) const
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
                if (std::next(m_listIter) == getOutNeighbors().end())
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
                m_mapRange(mapRange), m_listIter(getOutNeighbors().begin()) {seekNextNotDeadEnd();}

            // TODO: Ensure that non-const `OtherValue` is not convertible to const `OtherValue`.
            template <typename OtherValue>
            EdgeIterator(const EdgeIterator<OtherValue>& other):
                m_mapRange(other.m_mapRange), m_listIter(other.m_listIter), m_current(other.m_current) {}
    };
}

#endif
