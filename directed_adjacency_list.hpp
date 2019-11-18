#ifndef GRAPHLIB_DIRECTED_ADJACENCY_LIST_HPP_1092019
#define GRAPHLIB_DIRECTED_ADJACENCY_LIST_HPP_1092019

#include <unordered_map>
#include <list>
#include <type_traits>
#include <limits>
#include <utility>
#include <functional>
#include <iterator>
#include <boost/optional.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/utility/result_of.hpp>

#include "constraints.hpp"
#include "edge_wrapper.hpp"

namespace graph
{
    template <typename Vertex, typename Edge = void, Constraints constraints = Constraints::SIMPLE,
        typename SizeType = unsigned int>
    class DirectedAdjacencyList final
    {
        private:
            template <typename Value>
            class AllEdgeIterator;

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
                std::list<adjInfo_t> outNeighbors;
                SizeType indegree = 0;
            };

            template <typename Iterator>
            using baseGetter_t = std::function<typename std::list<adjInfo_t>::const_iterator(Iterator&)>;

            // facilitates the use of iterators to remove edges efficiently, via `std::list.erase(const_iterator)`
            template <typename Iterator>
            class EdgeIteratorWrapper final: public boost::iterator_adaptor<EdgeIteratorWrapper<Iterator>, Iterator>
            {
                private:
                    baseGetter_t<Iterator> m_baseGetter;

                public:
                    EdgeIteratorWrapper() = default;
                    EdgeIteratorWrapper(const Iterator& iter, baseGetter_t<Iterator> baseGetter):
                        EdgeIteratorWrapper::iterator_adaptor_(iter), m_baseGetter(baseGetter) {}

                    typename std::list<adjInfo_t>::const_iterator deepBase()
                    {
                        return m_baseGetter(EdgeIteratorWrapper::iterator_adaptor_::base_reference());
                    }
            };

            template <typename Iterator>
            static EdgeIteratorWrapper<Iterator> makeEdgeIteratorWrapper(const Iterator& iter,
                baseGetter_t<Iterator> baseGetter)
            {
                return EdgeIteratorWrapper<Iterator>(iter, baseGetter);
            }

            // Arguments to `edgeWrapper` should always be l-values.

            template <typename E, typename A, std::enable_if_t<std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<const Vertex, E> edgeWrapper(const Vertex& tail, A& adjInfo)
            {
                return {&tail, &adjInfo.vertex};
            }

            template <typename E, typename A, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<const Vertex, E> edgeWrapper(const Vertex& tail, A& adjInfo)
            {
                static_assert(std::is_same<typename std::remove_const_t<E>, Edge>::value);
                return {&tail, &adjInfo.vertex, &adjInfo.edge};
            }

            static auto implOutNeighbors(std::pair<const Vertex, TailInfo>& edgeInfo)
            {
                std::function<const edgeWrapper_t<const Vertex, Edge>(adjInfo_t&)> transform(
                    [&](adjInfo_t& adjInfo){return edgeWrapper<Edge>(edgeInfo.first, adjInfo);});
                std::list<adjInfo_t>& outNeighbors = edgeInfo.second.outNeighbors;
                auto begin(boost::make_transform_iterator(outNeighbors.begin(), transform));
                baseGetter_t<decltype(begin)> baseGetter(&decltype(begin)::base);
                return std::make_pair(
                    makeEdgeIteratorWrapper(begin, baseGetter),
                    makeEdgeIteratorWrapper(
                        boost::make_transform_iterator(outNeighbors.end(), transform), baseGetter));
            }

            static auto implConstOutNeighbors(const std::pair<const Vertex, TailInfo>& edgeInfo)
            {
                std::function<const edgeWrapper_t<const Vertex, const Edge>(const adjInfo_t&)> transform(
                    [&](const adjInfo_t& adjInfo){return edgeWrapper<const Edge>(edgeInfo.first, adjInfo);});
                const std::list<adjInfo_t>& outNeighbors = edgeInfo.second.outNeighbors;
                return std::make_pair(
                    boost::make_transform_iterator(outNeighbors.begin(), transform),
                    boost::make_transform_iterator(outNeighbors.end(), transform));
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
            auto implInNeighbors(const Vertex& head)
            {
                std::function<bool(const edgeWrapper_t<const Vertex, Edge>&)> pred(
                    [&](const edgeWrapper_t<const Vertex, Edge>& wrapper){return *wrapper.head == head;});
                auto mapRange(std::make_pair(m_adjMap.begin(), m_adjMap.end()));
                auto begin(boost::make_filter_iterator(pred,
                    AllEdgeIterator<edgeWrapper_t<const Vertex, Edge>>(mapRange)));
                baseGetter_t<decltype(begin)> baseGetter([](decltype(begin)& iter){return iter.base().deepBase();});
                return std::make_pair(
                    makeEdgeIteratorWrapper(begin, baseGetter),
                    makeEdgeIteratorWrapper(boost::make_filter_iterator(pred,
                        AllEdgeIterator<edgeWrapper_t<const Vertex, Edge>>()), baseGetter));
            }

            auto implConstInNeighbors(const Vertex& head) const
            {
                std::function<bool(const edgeWrapper_t<const Vertex, const Edge>&)> pred(
                    [&](const edgeWrapper_t<const Vertex, const Edge>& wrapper){return *wrapper.head == head;});
                auto mapRange(std::make_pair(m_adjMap.begin(), m_adjMap.end()));
                return std::make_pair(
                    boost::make_filter_iterator(pred,
                        AllEdgeIterator<edgeWrapper_t<const Vertex, const Edge>>(mapRange)),
                    boost::make_filter_iterator(pred, AllEdgeIterator<edgeWrapper_t<const Vertex, const Edge>>()));
            }

            template <typename V>
            bool implAddVertex(V&& vertex)
            {
                if (m_adjMap.count(vertex))
                    return false;
                else
                    return m_adjMap.emplace(std::forward<V>(vertex), TailInfo{}).second;
            }

            template <typename V1, typename V2>
            adjInfo_t& addEndvertices(V1&& tail, V2&& head)
            {
                auto tailIter(m_adjMap.find(tail));
                if (tailIter == m_adjMap.end())
                    tailIter = m_adjMap.emplace(std::forward<V1>(tail), TailInfo{}).first;
                auto headIter(m_adjMap.find(head));
                if (headIter == m_adjMap.end())
                    headIter = m_adjMap.emplace(std::forward<V2>(head), TailInfo{}).first;
                TailInfo& tailInfo = tailIter->second;
                ++m_size, ++headIter->second.indegree;
                tailInfo.outNeighbors.push_back({head});
                return tailInfo.outNeighbors.back();
            }

            template <typename V1, typename V2, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) == 0 && std::is_same<V1, V1>::value, bool> = true>
            adjInfo_t& addEndverticesIfValid(V1&& tail, V2&& head)
            {
                return addEndvertices(std::forward<V1>(tail), std::forward<V2>(head));
            }

            template <typename V1, typename V2, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) != 0 && std::is_same<V1, V1>::value, bool> = true>
            adjInfo_t& addEndverticesIfValid(V1&& tail, V2&& head)
            {
                if (tail == head)
                    throw ConstraintViolationException(Constraints::NO_LOOPS);
                return addEndvertices(std::forward<V1>(tail), std::forward<V2>(head));
            }

            template <typename Predicate>
            SizeType implRemoveEdges(const Vertex& tail, const Vertex& head, SizeType n, Predicate&& pred)
            {
                auto tailIter(m_adjMap.find(tail));
                if (tailIter == m_adjMap.end())
                    return 0;
                std::list<adjInfo_t>& outNeighbors = tailIter->second.outNeighbors;
                SizeType k = 0;
                auto iter(outNeighbors.begin());
                while (k < n && iter != outNeighbors.end())
                {
                    if (pred(tail, head, *iter))
                    {
                        --m_adjMap.at(iter->vertex).indegree;
                        iter = outNeighbors.erase(iter);
                        ++k;
                    }
                    else
                        ++iter;
                }
                m_size -= k;
                return k;
            }

        public:
            auto vertices() const
            {
                const Vertex& (*transform)(const std::pair<const Vertex, TailInfo>&) = std::get<const Vertex, TailInfo>;
                return std::make_pair(
                    boost::make_transform_iterator(m_adjMap.begin(), transform),
                    boost::make_transform_iterator(m_adjMap.end(), transform));
            }

            using vertexIter_t =
                decltype(typename boost::result_of<decltype(&DirectedAdjacencyList::vertices)()>::type().first);

            using allEdgeIter_t = AllEdgeIterator<edgeWrapper_t<const Vertex, Edge>>;

            auto edges()
            {
                return std::make_pair(allEdgeIter_t(std::make_pair(m_adjMap.begin(), m_adjMap.end())), allEdgeIter_t());
            }

            using constAllEdgeIter_t = AllEdgeIterator<edgeWrapper_t<const Vertex, const Edge>>;

            auto edges() const
            {
                return std::make_pair(
                    constAllEdgeIter_t(std::make_pair(m_adjMap.begin(), m_adjMap.end())), constAllEdgeIter_t());
            }

            auto constEdges() const {return edges();}

            using inNeighborIter_t = decltype(typename boost::result_of<
                decltype(&DirectedAdjacencyList::implInNeighbors)(const Vertex&)>::type().first);

            auto inNeighbors(const Vertex& head)
            {
                return m_adjMap.count(head)
                    ? implInNeighbors(head)
                    : std::pair<inNeighborIter_t, inNeighborIter_t>();
            }

            using constInNeighborIter_t = decltype(typename boost::result_of<
                decltype(&DirectedAdjacencyList::implConstInNeighbors)(const Vertex&)>::type().first);

            auto inNeighbors(const Vertex& head) const
            {
                return m_adjMap.count(head)
                    ? implConstInNeighbors(head)
                    : std::pair<constInNeighborIter_t, constInNeighborIter_t>();
            }

            auto constInNeighbors(const Vertex& head) const {return inNeighbors(head);}

            using outNeighborIter_t = decltype(typename boost::result_of<
                decltype(&DirectedAdjacencyList::implOutNeighbors)(std::pair<const Vertex, TailInfo>&)>::type().first);

            auto outNeighbors(const Vertex& tail)
            {
                auto tailIter(m_adjMap.find(tail));
                return tailIter != m_adjMap.end()
                    ? implOutNeighbors(*tailIter)
                    : std::pair<outNeighborIter_t, outNeighborIter_t>();
            }

            using constOutNeighborIter_t = decltype(typename boost::result_of<
                decltype(&DirectedAdjacencyList::implConstOutNeighbors)(
                    const std::pair<const Vertex, TailInfo>&)>::type().first);

            auto outNeighbors(const Vertex& tail) const
            {
                auto tailIter(m_adjMap.find(tail));
                return tailIter != m_adjMap.end()
                    ? implConstOutNeighbors(*tailIter)
                    : std::pair<constOutNeighborIter_t, constOutNeighborIter_t>();
            }

            auto constOutNeighbors(const Vertex& tail) const {return outNeighbors(tail);}

            SizeType order() const {return m_adjMap.size();}
            SizeType size() const {return m_size;}

            SizeType indegree(const Vertex& vertex) const {return m_adjMap.at(vertex).indegree;}
            SizeType indegree(const vertexIter_t& iter) const {return iter.base()->second.indegree;}
            SizeType outdegree(const Vertex& vertex) const {return m_adjMap.at(vertex).outNeighbors.size();}
            SizeType outdegree(const vertexIter_t& iter) const {return iter.base()->second.outNeighbors.size();}

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

            template <typename V, std::enable_if_t<
                (constraints & Constraints::CONNECTED) == 0 && std::is_same<V, V>::value, bool> = true>
            bool addVertex(V&& vertex) {return implAddVertex(std::forward<V>(vertex));}

            template <typename V, std::enable_if_t<
                (constraints & Constraints::CONNECTED) != 0 && std::is_same<V, V>::value, bool> = true>
            bool addVertex(V&& vertex)
            {
                // The null graph on one vertex is connected.
                if (!order() || contains(vertex))
                    return implAddVertex(std::forward<V>(vertex));
                else
                    throw ConstraintViolationException(Constraints::CONNECTED);
            }

            template <typename V1, typename V2>
            void addEdge(V1&& tail, V2&& head) {addEndverticesIfValid(std::forward<V1>(tail), std::forward<V2>(head));}

            template <typename V1, typename V2, typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            void addEdge(V1&& tail, V2&& head, E&& edge)
            {
                addEndverticesIfValid(std::forward<V1>(tail), std::forward<V2>(head)).edge = edge;
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
                return implRemoveEdges(tail, head, n, std::move(pred));
            }

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            SizeType removeEdges(const Vertex& tail, const Vertex& head, const E& edge, SizeType n)
            {
                auto pred = [&](const Vertex& tail, const Vertex& head, const adjInfo_t& adjInfo)
                {
                    return head == adjInfo.vertex && edge == adjInfo.edge;
                };
                return implRemoveEdges(tail, head, n, std::move(pred));
            }

            bool removeEdge(const Vertex& tail, const Vertex& head) {return removeEdges(tail, head, 1);}

            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            bool removeEdge(const Vertex& tail, const Vertex& head, const E& edge)
            {
                return removeEdges(tail, head, edge, 1);
            }

            template <typename EdgeIterator>
            void removeEdge(EdgeIterator&& iter)
            {
                auto baseIter(iter.deepBase());
                auto wrapper(*iter);
                TailInfo& tailInfo = m_adjMap.at(*wrapper.tail);
                --m_adjMap.at(*wrapper.head).indegree;
                --m_size;
                ++iter;
                tailInfo.outNeighbors.erase(baseIter);
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

            void clear()
            {
                m_adjMap.clear();
                m_size = 0;
            }
    };

    /*
     * Almost conforms to `LegacyForwardIterator`. There is only one deviation and it is insignificant in most contexts.
     * For two instances `a` and `b`, it is possible that `a == b && &*a != &*b`. However, for each pair of members
     * `a->m` and `b->m`, if `a == b` then `a->m == b->m`.
     *
     * `Value` is always either `edgeWrapper_t<const Vertex, Edge>` or `edgeWrapper_t<const Vertex, const Edge>`.
     */
    template <typename Vertex, typename Edge, Constraints constraints, typename SizeType>
    template <typename Value>
    class DirectedAdjacencyList<Vertex, Edge, constraints, SizeType>::AllEdgeIterator final:
        public boost::iterator_facade<
            DirectedAdjacencyList<Vertex, Edge, constraints, SizeType>::AllEdgeIterator<Value>,
            const Value, std::input_iterator_tag>
    {
        friend class boost::iterator_core_access;

        template <typename>
        friend class AllEdgeIterator;

        private:
            using mapIter_t = std::conditional_t<
                std::is_const<typename Value::edge_t>::value || std::is_void<typename Value::edge_t>::value,
                typename decltype(m_adjMap)::const_iterator, typename decltype(m_adjMap)::iterator>;
            using listIter_t = std::conditional_t<
                std::is_const<typename Value::edge_t>::value || std::is_void<typename Value::edge_t>::value,
                typename std::list<adjInfo_t>::const_iterator, typename std::list<adjInfo_t>::iterator>;

            std::pair<mapIter_t, mapIter_t> m_mapRange;
            std::pair<listIter_t, listIter_t> m_listRange;
            boost::optional<Value> m_current;

            bool findNext()
            {
                if (m_listRange.first == m_listRange.second)
                {
                    while (++m_mapRange.first != m_mapRange.second)
                    {
                        auto& outNeighbors = m_mapRange.first->second.outNeighbors;
                        if (outNeighbors.size())
                        {
                            m_listRange.first = outNeighbors.begin(), m_listRange.second = outNeighbors.end();
                            return true;
                        }
                    }
                    return false;
                }
                else
                    return true;
            }

            const Value& dereference() const {return *m_current;}

            template <typename OtherValue>
            bool equal(const AllEdgeIterator<OtherValue>& other) const {return m_current == other.m_current;}

            void increment()
            {
                ++m_listRange.first;
                m_current = findNext()
                    ? edgeWrapper<typename Value::edge_t>(m_mapRange.first->first, *m_listRange.first)
                    : boost::optional<Value>();
            }

        public:
            AllEdgeIterator() = default;
            AllEdgeIterator(const AllEdgeIterator&) = default;
            AllEdgeIterator(AllEdgeIterator&&) = default;

            explicit AllEdgeIterator(const std::pair<mapIter_t, mapIter_t>& mapRange): m_mapRange(mapRange)
            {
                if (m_mapRange.first != m_mapRange.second)
                {
                    auto& outNeighbors = m_mapRange.first->second.outNeighbors;
                    m_listRange.first = outNeighbors.begin(), m_listRange.second = outNeighbors.end();
                    if (findNext())
                        m_current = edgeWrapper<typename Value::edge_t>(m_mapRange.first->first, *m_listRange.first);
                }
            }

            // is const because boost's `base()` is often const
            listIter_t deepBase() const {return m_listRange.first;}

            AllEdgeIterator& operator=(const AllEdgeIterator&) = default;
            AllEdgeIterator& operator=(AllEdgeIterator&&) = default;

            // The following constructors and assignment operators allow conversion from non-const iterators to
            // const iterators.

            template <typename T = allEdgeIter_t>
            AllEdgeIterator& operator=(
                const std::enable_if_t<!std::is_same<AllEdgeIterator, T>::value, T>& other)
            {

                m_mapRange = other.m_mapRange;
                m_listRange = other.m_listRange;
                m_current = other.m_current;
                return *this;
            }

            template <typename T = allEdgeIter_t>
            AllEdgeIterator& operator=(std::enable_if_t<!std::is_same<AllEdgeIterator, T>::value, T>&& other)
            {
                m_mapRange = std::move(other.m_mapRange);
                m_listRange = std::move(other.m_listRange);
                m_current = std::move(other.m_current);
                return *this;
            }

            template <typename T = allEdgeIter_t>
            AllEdgeIterator(const std::enable_if_t<!std::is_same<AllEdgeIterator, T>::value, T>& other) {*this = other;}

            template <typename T = allEdgeIter_t>
            AllEdgeIterator(std::enable_if_t<!std::is_same<AllEdgeIterator, T>::value, T>&& other) {*this = other;}
    };
}

#endif
