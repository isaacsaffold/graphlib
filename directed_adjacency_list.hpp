#ifndef GRAPH_DIRECTED_ADJACENCY_LIST_HPP_1092019
#define GRAPH_DIRECTED_ADJACENCY_LIST_HPP_1092019

#include <cstddef>
#include <unordered_map>
#include <forward_list>
#include <type_traits>
#include <limits>
#include <utility>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/utility/result_of.hpp>

#include "constraints.hpp"
#include "edge_wrapper.hpp"

namespace graph
{
    template <typename Vertex, typename Edge = void, Constraints constraints = Constraints::SIMPLY_CONNECTED,
        typename SizeType = std::size_t>
    class DirectedAdjacencyList final
    {
        private:
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

            struct TailInfo
            {
                std::forward_list<adjInfo_t> outNeighbors;
                SizeType indegree = 0, outdegree = 0;
            };

            // Arguments to `edgeWrapper` should always be l-values.

            template <typename E, std::enable_if_t<std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<const Vertex, E> edgeWrapper(const Vertex& tail, const adjInfo_t& adjInfo)
            {
                return {&tail, &adjInfo.vertex};
            }

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<const Vertex, E> edgeWrapper(const Vertex& tail, const adjInfo_t& adjInfo)
            {
                static_assert(std::is_same<typename std::remove_const_t<E>, Edge>::value);
                return {&tail, &adjInfo.vertex, &adjInfo.edge};
            }

        public:
            static constexpr Constraints CONSTRAINTS = constraints;
            static constexpr bool IS_DIRECTED = true;

            using vertex_t = Vertex;
            using edge_t = Edge;
            using customSize_t = SizeType;

        private:
            std::unordered_map<Vertex, TailInfo> m_adjMap;
            SizeType m_size = 0;

        public:
            DirectedAdjacencyList() = default;
            explicit DirectedAdjacencyList(SizeType initialCapacity): m_adjMap(initialCapacity) {}

        private:
            template <typename V>
            adjInfo_t& addEndvertices(V&& tail, V&& head)
            {
                auto tailIter(m_adjMap.find(tail));
                if (tailIter == m_adjMap.end())
                    tailIter = m_adjMap.emplace(tail, TailInfo{}).first;
                auto headIter(m_adjMap.find(head));
                if (headIter == m_adjMap.end())
                    headIter = m_adjMap.emplace(head, TailInfo{}).first;
                TailInfo& tailInfo = tailIter->second;
                ++m_size, ++tailInfo.outdegree, ++headIter->second.indegree;
                return *tailInfo.outNeighbors.insert_after(tailInfo.outNeighbors.before_begin(), {head});
            }

            template <typename V, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) == 0 && std::is_same<V, V>::value, bool> = true>
            adjInfo_t& addEndverticesIfValid(V&& tail, V&& head)
            {
                return addEndvertices(tail, head);
            }

            template <typename V, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) != 0 && std::is_same<V, V>::value, bool> = true>
            adjInfo_t& addEndverticesIfValid(V&& tail, V&& head)
            {
                if (tail == head)
                    throw ConstraintViolationException(Constraints::NO_LOOPS);
                return addEndvertices(tail, head);
            }

            template <typename Predicate>
            SizeType implRemoveEdges(const Vertex& tail, const Vertex& head, SizeType n, Predicate pred)
            {
                auto tailIter(m_adjMap.find(tail));
                if (tailIter == m_adjMap.end())
                    return 0;
                std::forward_list<adjInfo_t>& outNeighbors = tailIter->second.outNeighbors;
                SizeType k = 0;
                auto before(outNeighbors.before_begin()), current(outNeighbors.begin());
                while (k < n && current != outNeighbors.end())
                {
                    if (pred(tail, head, *current))
                    {
                        --tailIter->second.outdegree;
                        --m_adjMap.at(current->vertex).indegree;
                        current = outNeighbors.erase_after(before);
                        ++k;
                    }
                    else
                        before = current++;
                }
                m_size -= k;
                return k;
            }

        public:
            SizeType order() const {return m_adjMap.size();}
            SizeType size() const {return m_size;}

            SizeType indegree(const Vertex& vertex) const {return m_adjMap.at(vertex).indegree;}
            SizeType outdegree(const Vertex& vertex) const {return m_adjMap.at(vertex).outdegree;}

            bool contains(const Vertex& vertex) const {return m_adjMap.count(vertex);}

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
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
                if (iter == m_adjMap.end())
                    return false;
                for (const adjInfo_t& adjInfo: iter->second.outNeighbors)
                {
                    if (head == adjInfo.vertex)
                        return true;
                }
                return false;
            }

            auto vertices() const
            {
                const Vertex& (*transform)(const std::pair<const Vertex, TailInfo>&) = std::get<const Vertex, TailInfo>;
                return std::make_pair(
                    boost::make_transform_iterator(m_adjMap.begin(), transform),
                    boost::make_transform_iterator(m_adjMap.end(), transform));
            }
            using vertexIter_t =
                decltype(typename boost::result_of<decltype(&DirectedAdjacencyList::vertices)()>::type().first);

            template <typename V, std::enable_if_t<
                (constraints & Constraints::CONNECTED) == 0 && std::is_same<V, V>::value, bool> = true>
            bool addVertex(V&& vertex)
            {
                if (m_adjMap.count(vertex))
                    return false;
                else
                    return m_adjMap.emplace(vertex, TailInfo{}).second;
            }

            template <typename V>
            void addEdge(V&& tail, V&& head) {addEndverticesIfValid(tail, head);}

            template <typename V, typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            void addEdge(V&& tail, V&& head, E&& edge)
            {
                addEndverticesIfValid(tail, head).edge = edge;
            }

            // Parameter `n` in `removeEdges` does not have a default because that would cause an ambiguity between
            // `removeEdges(Vertex, Vertex, SizeType)` and `removeEdges(Vertex, Vertex, Edge)` when `Edge` and
            // `SizeType` are the same, and other permutations of the parameters could also cause ambiguities.

            SizeType removeEdges(const Vertex& tail, const Vertex& head, SizeType n)
            {
                auto pred = [](const Vertex& tail, const Vertex& head, const adjInfo_t& adjInfo)
                {
                    return head == adjInfo.vertex;
                };
                return implRemoveEdges(tail, head, n, pred);
            }

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            SizeType removeEdges(const Vertex& tail, const Vertex& head, const E& edge, SizeType n)
            {
                auto pred = [&](const Vertex& tail, const Vertex& head, const adjInfo_t& adjInfo)
                {
                    return head == adjInfo.vertex && edge == adjInfo.edge;
                };
                return implRemoveEdges(tail, head, n, pred);
            }

            bool removeEdge(const Vertex& tail, const Vertex& head) {return removeEdges(tail, head, 1);}

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            bool removeEdge(const Vertex& tail, const Vertex& head, const E& edge)
            {
                return removeEdges(tail, head, edge, 1);
            }

            SizeType removeAllEdges(const Vertex& tail, const Vertex& head)
            {
                return removeEdges(tail, head, std::numeric_limits<SizeType>::max());
            }

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            SizeType removeAllEdges(const Vertex& tail, const Vertex& head, const E& edge)
            {
                return removeEdges(tail, head, edge, std::numeric_limits<SizeType>::max());
            }

            bool removeVertex(const Vertex& vertex)
            {
                auto iter(m_adjMap.find(vertex));
                if (iter == m_adjMap.end())
                    return false;
                for (adjInfo_t& adjInfo: iter->second.outNeighbors)
                    --m_adjMap.at(adjInfo.vertex).indegree, --m_size;
                for (auto& elem: m_adjMap)
                    removeAllEdges(elem.first, vertex);
                m_adjMap.erase(iter);
                return true;
            }
    };
}

#endif
