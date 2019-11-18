#ifndef GRAPHLIB_UNDIRECTED_ADJACENCY_LIST_HPP_11142019
#define GRAPHLIB_UNDIRECTED_ADJACENCY_LIST_HPP_11142019

#include <unordered_map>
#include <list>
#include <type_traits>
#include <memory>
#include <utility>
#include <array>
#include <iterator>
#include <functional>
#include <boost/optional.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/iterator/iterator_adaptor.hpp>
#include <boost/utility/result_of.hpp>

#include "constraints.hpp"
#include "edge_wrapper.hpp"

// TODO: Put duplicate members into a base class or make them global and available to both classes.

namespace graph
{
    template <typename Vertex, typename Edge = void, Constraints constraints = Constraints::SIMPLE,
        typename SizeType = unsigned int>
    class UndirectedAdjacencyList final
    {
        private:
            // duplicate
            template <typename Value>
            class AllEdgeIterator;

            // duplicate
            struct ImplicitAdjacencyInfo
            {
                Vertex vertex;
            };

            // duplicate
            struct AdjacencyInfo
            {
                Vertex vertex;
                std::shared_ptr<Edge> edge;
            };

            // duplicate
            using adjInfo_t = std::conditional_t<std::is_void<Edge>::value, ImplicitAdjacencyInfo, AdjacencyInfo>;

            struct AdjInfoWrapper
            {
                adjInfo_t wrapped;
                bool marked;
                typename std::list<AdjInfoWrapper>::const_iterator twin;
            };

            // duplicate
            template <typename Iterator>
            using baseGetter_t = std::function<typename std::list<AdjInfoWrapper>::const_iterator(Iterator&)>;

            // duplicate
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

                    typename std::list<AdjInfoWrapper>::const_iterator deepBase()
                    {
                        return m_baseGetter(EdgeIteratorWrapper::iterator_adaptor_::base_reference());
                    }
            };

            // duplicate
            template <typename Iterator>
            static EdgeIteratorWrapper<Iterator> makeEdgeIteratorWrapper(const Iterator& iter,
                baseGetter_t<Iterator> baseGetter)
            {
                return EdgeIteratorWrapper<Iterator>(iter, baseGetter);
            }

            // Arguments to `edgeWrapper` should always be l-values.

            template <typename E, typename A, std::enable_if_t<std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<const Vertex, E> edgeWrapper(const Vertex& vertex, A& adjWrapper)
            {
                return {&vertex, &adjWrapper.wrapped.vertex};
            }

            template <typename E, typename A, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            static edgeWrapper_t<const Vertex, E> edgeWrapper(const Vertex& vertex, A& adjWrapper)
            {
                static_assert(std::is_same<typename std::remove_const_t<E>, Edge>::value);
                return {&vertex, &adjWrapper.wrapped.vertex, adjWrapper.wrapped.edge.get()};
            }

            // duplicate
            static auto implNeighbors(std::pair<const Vertex, std::list<AdjInfoWrapper>>& edgeInfo)
            {
                std::function<const edgeWrapper_t<const Vertex, Edge>(AdjInfoWrapper&)> transform(
                    [&](AdjInfoWrapper& adjWrapper){return edgeWrapper<Edge>(edgeInfo.first, adjWrapper);});
                auto begin(boost::make_transform_iterator(edgeInfo.second.begin(), transform));
                baseGetter_t<decltype(begin)> baseGetter(&decltype(begin)::base);
                return std::make_pair(
                    makeEdgeIteratorWrapper(begin, baseGetter),
                    makeEdgeIteratorWrapper(
                        boost::make_transform_iterator(edgeInfo.second.end(), transform), baseGetter));
            }

            // duplicate
            static auto implConstNeighbors(const std::pair<const Vertex, std::list<AdjInfoWrapper>>& edgeInfo)
            {
                std::function<const edgeWrapper_t<const Vertex, const Edge>(const AdjInfoWrapper&)> transform(
                    [&](const AdjInfoWrapper& adjWrapper){return edgeWrapper<const Edge>(edgeInfo.first, adjWrapper);});
                return std::make_pair(
                    boost::make_transform_iterator(edgeInfo.second.begin(), transform),
                    boost::make_transform_iterator(edgeInfo.second.end(), transform));
            }

        public:
            static constexpr Constraints CONSTRAINTS = constraints;
            static constexpr bool IS_DIRECTED = false;

            using vertex_t = Vertex;
            using edge_t = Edge;
            using customSize_t = SizeType;

        private:
            std::unordered_map<Vertex, std::list<AdjInfoWrapper>> m_adjMap;
            // duplicate
            SizeType m_size = 0;

        public:
            UndirectedAdjacencyList() = default;
            explicit UndirectedAdjacencyList(SizeType initialCapacity): m_adjMap(initialCapacity) {}

        private:
            // duplicate
            template <typename V>
            bool implAddVertex(V&& vertex)
            {
                if (m_adjMap.count(vertex))
                    return false;
                else
                    return m_adjMap.emplace(std::forward<V>(vertex), std::list<AdjInfoWrapper>()).second;
            }

            // duplicate
            template <typename V1, typename V2>
            std::array<std::list<AdjInfoWrapper>*, 2> addEndvertices(V1&& first, V2&& second)
            {
                auto firstIter(m_adjMap.find(first));
                if (firstIter == m_adjMap.end())
                    firstIter = m_adjMap.emplace(std::forward<V1>(first), std::list<AdjInfoWrapper>()).first;
                auto secondIter(m_adjMap.find(second));
                if (secondIter == m_adjMap.end())
                    secondIter = m_adjMap.emplace(std::forward<V2>(second), std::list<AdjInfoWrapper>()).first;
                return {&firstIter->second, &secondIter->second};
            }

            // duplicate
            template <typename V1, typename V2, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) == 0 && std::is_same<V1, V1>::value, bool> = true>
            std::array<std::list<AdjInfoWrapper>*, 2> addEndverticesIfValid(V1&& first, V2&& second)
            {
                return addEndvertices(std::forward<V1>(first), std::forward<V2>(second));
            }

            // duplicate
            template <typename V1, typename V2, std::enable_if_t<
                (constraints & Constraints::NO_LOOPS) != 0 && std::is_same<V1, V1>::value, bool> = true>
            std::array<std::list<AdjInfoWrapper>*, 2> addEndverticesIfValid(V1&& first, V2&& second)
            {
                if (first == second)
                    throw ConstraintViolationException(Constraints::NO_LOOPS);
                return addEndvertices(std::forward<V1>(first), std::forward<V2>(second));
            }

            template <typename V1, typename V2>
            void implAddEdge(V1&& first, V2&& second, adjInfo_t&& toFirst, adjInfo_t&& toSecond)
            {
                auto neighborLists(addEndverticesIfValid(std::forward<V1>(first), std::forward<V2>(second)));
                auto firstIter(neighborLists[0]->insert(neighborLists[0]->cend(), {toFirst, true}));
                auto secondIter(neighborLists[1]->insert(
                    neighborLists[1]->cend(), {toSecond, false, firstIter}));
                firstIter->twin = secondIter;
                ++m_size;
            }

        public:
            // duplicate
            auto vertices() const
            {
                const Vertex& (*transform)(const std::pair<const Vertex, std::list<AdjInfoWrapper>>&) =
                    std::get<const Vertex, std::list<AdjInfoWrapper>>;
                return std::make_pair(
                    boost::make_transform_iterator(m_adjMap.begin(), transform),
                    boost::make_transform_iterator(m_adjMap.end(), transform));
            }

            // duplicate
            using vertexIter_t =
                decltype(typename boost::result_of<decltype(&UndirectedAdjacencyList::vertices)()>::type().first);

            // duplicate
            using allEdgeIter_t = AllEdgeIterator<edgeWrapper_t<const Vertex, Edge>>;

            // duplicate
            auto edges()
            {
                return std::make_pair(allEdgeIter_t(std::make_pair(m_adjMap.begin(), m_adjMap.end())), allEdgeIter_t());
            }

            // duplicate
            using constAllEdgeIter_t = AllEdgeIterator<edgeWrapper_t<const Vertex, const Edge>>;

            // duplicate
            auto edges() const
            {
                return std::make_pair(
                    constAllEdgeIter_t(std::make_pair(m_adjMap.begin(), m_adjMap.end())), constAllEdgeIter_t());
            }

            // duplicate
            auto constEdges() const {return edges();}

            // duplicate
            using neighborIter_t = decltype(typename boost::result_of<
                decltype(&UndirectedAdjacencyList::implNeighbors)(
                    typename decltype(m_adjMap)::value_type&)>::type().first);

            // duplicate
            auto neighbors(const Vertex& vertex)
            {
                auto iter(m_adjMap.find(vertex));
                return iter != m_adjMap.end()
                    ? implNeighbors(*iter)
                    : std::pair<neighborIter_t, neighborIter_t>();
            }

            // duplicate
            using constNeighborIter_t = decltype(typename boost::result_of<
                decltype(&UndirectedAdjacencyList::implConstNeighbors)(
                    const typename decltype(m_adjMap)::value_type&)>::type().first);

            // duplicate
            auto neighbors(const Vertex& vertex) const
            {
                auto iter(m_adjMap.find(vertex));
                return iter != m_adjMap.end()
                    ? implConstNeighbors(*iter)
                    : std::pair<constNeighborIter_t, constNeighborIter_t>();
            }

            // duplicate
            auto constNeighbors(const Vertex& tail) const {return neighbors(tail);}

            // duplicate
            SizeType order() const {return m_adjMap.size();}
            // duplicate
            SizeType size() const {return m_size;}

            // duplicate
            SizeType degree(const Vertex& vertex) const {return m_adjMap.at(vertex).size();}
            // duplicate
            SizeType degree(const vertexIter_t& iter) const {return iter.base()->second.size();}

            // duplicate
            bool contains(const Vertex& vertex) const {return m_adjMap.count(vertex);}

            // duplicate
            template <typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            bool contains(const E& edge) const
            {
                for (const auto& elem: m_adjMap)
                {
                    for (const AdjInfoWrapper& adjInfo: elem.second)
                    {
                        if (edge == *adjInfo.wrapped)
                            return true;
                    }
                }
                return false;
            }

            // duplicate
            // if `first` or `second` are not contained in the graph, returns `false`
            bool areAdjacent(const Vertex& first, const Vertex& second) const
            {
                auto iter(m_adjMap.find(first));
                if (iter == m_adjMap.end())
                    return false;
                for (const AdjInfoWrapper& adjWrapper: iter->second)
                {
                    if (second == adjWrapper.wrapped.vertex)
                        return true;
                }
                return false;
            }

            // duplicate
            template <typename V, std::enable_if_t<
                (constraints & Constraints::CONNECTED) == 0 && std::is_same<V, V>::value, bool> = true>
            bool addVertex(V&& vertex) {return implAddVertex(std::forward<V>(vertex));}

            // duplicate
            template <typename V, std::enable_if_t<
                (constraints & Constraints::CONNECTED) != 0 && std::is_same<V, V>::value, bool> = true>
            bool addVertex(V&& vertex)
            {
                if (!order() || contains(vertex))
                    return implAddVertex(std::forward<V>(vertex));
                else
                    throw ConstraintViolationException(Constraints::CONNECTED);
            }

            template <typename V1, typename V2>
            void addEdge(V1&& first, V2&& second)
            {
                implAddEdge(std::forward<V1>(first), std::forward<V2>(second), {second}, {first});
            }

            template <typename V1, typename V2, typename E, std::enable_if_t<!std::is_void<E>::value, bool> = true>
            void addEdge(V1&& first, V2&& second, E&& edge)
            {
                auto ptr(std::make_shared<std::remove_const_t<std::remove_reference_t<E>>>(edge));
                implAddEdge(std::forward<V1>(first), std::forward<V2>(second), {second, ptr}, {first, ptr});
            }

            // duplicate
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
    class UndirectedAdjacencyList<Vertex, Edge, constraints, SizeType>::AllEdgeIterator final:
        public boost::iterator_facade<
            UndirectedAdjacencyList<Vertex, Edge, constraints, SizeType>::AllEdgeIterator<Value>,
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
                typename std::list<AdjInfoWrapper>::const_iterator, typename std::list<AdjInfoWrapper>::iterator>;

            std::pair<mapIter_t, mapIter_t> m_mapRange;
            std::pair<listIter_t, listIter_t> m_listRange;
            boost::optional<Value> m_current;

            bool findNext()
            {
                if (m_listRange.first != m_listRange.second && m_listRange.first->marked)
                    return true;
                while (true)
                {
                    while (m_listRange.first == m_listRange.second)
                    {
                        if (++m_mapRange.first == m_mapRange.second)
                            return false;
                        m_listRange.first = m_mapRange.first->second.begin();
                        m_listRange.second = m_mapRange.first->second.end();
                    }
                    for (; m_listRange.first != m_listRange.second; ++m_listRange.first)
                    {
                        if (m_listRange.first->marked)
                            return true;
                    }
                }
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
                    auto& neighbors = m_mapRange.first->second;
                    m_listRange.first = neighbors.begin(), m_listRange.second = neighbors.end();
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
