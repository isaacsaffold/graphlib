#ifndef GRAPH_ALGORITHMS_HPP_9192019
#define GRAPH_ALGORITHMS_HPP_9192019

#include <queue>
#include <stack>
#include <iterator>
#include <unordered_set>
#include <utility>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <boost/iterator/iterator_facade.hpp>

namespace graph
{
    #if __cplusplus < 201703
    template <typename F, typename... ArgsTypes>
    using return_t = std::result_of<F, ArgsTypes...>;
    #else
    template <typename F, typename... ArgsTypes>
    using return_t = std::invoke_result<F, ArgsTypes...>;
    #endif

    template <typename VertexType, typename DirectedGraph>
    class DirectedBreadthFirstIterator;
    template <typename VertexType, typename DirectedGraph>
    class DirectedDepthFirstIterator;

    template <typename VertexType>
    using visitedPtr_t = std::shared_ptr<std::unordered_set<VertexType>>;

    template <typename VertexType, typename DirectedGraph>
    auto directedBreadthFirst(const DirectedGraph& digraph, const VertexType& start,
        const visitedPtr_t<VertexType>& visited = std::make_shared<std::unordered_set<VertexType>>())
    {
        return std::make_pair(
            DirectedBreadthFirstIterator<VertexType, DirectedGraph>(&digraph, start, visited),
            DirectedBreadthFirstIterator<VertexType, DirectedGraph>());
    }

    template <typename VertexType, typename DirectedGraph>
    class DirectedBreadthFirstIterator final: public boost::iterator_facade<
        DirectedBreadthFirstIterator<VertexType, DirectedGraph>, const VertexType, std::input_iterator_tag>
    {
        friend class boost::iterator_core_access;
        friend auto directedBreadthFirst<VertexType, DirectedGraph>(
            const DirectedGraph&, const VertexType&, const visitedPtr_t<VertexType>&);

        private:
            const DirectedGraph* m_graph = nullptr;
            VertexType m_start;
            std::shared_ptr<std::queue<VertexType>> m_vertexQueue;
            visitedPtr_t<VertexType> m_visited;
            std::size_t m_poppedCount = 0;

            DirectedBreadthFirstIterator(const DirectedGraph* digraph, const VertexType& start,
                const visitedPtr_t<VertexType>& visited):
                m_graph(digraph), m_start(start), m_vertexQueue(new std::queue<VertexType>),
                m_visited(visited)
            {
                m_vertexQueue->push(start);
                m_visited->insert(start);
            }

            const VertexType& dereference() const {return m_vertexQueue->front();}

            bool equal(const DirectedBreadthFirstIterator<VertexType, DirectedGraph>& other) const
            {
                bool otherQueueIsEmpty = !other.m_vertexQueue || !other.m_vertexQueue->size();
                if (!m_vertexQueue || !m_vertexQueue->size())
                    return otherQueueIsEmpty;
                else if (otherQueueIsEmpty)
                    return false;
                // If they start with the same vertex and have traversed the same number of vertices, they are equal.
                else
                    return m_graph == other.m_graph && m_start == other.m_start && m_poppedCount == other.m_poppedCount;
            }

            void increment()
            {
                auto iters(m_graph->outNeighbors(m_vertexQueue->front()));
                for (; iters.first != iters.second; ++iters.first)
                {
                    if (!m_visited->count(*iters.first))
                    {
                        m_vertexQueue->push(*iters.first);
                        m_visited->insert(*iters.first);
                    }
                }
                m_vertexQueue->pop();
                ++m_poppedCount;
            }

        public:
            DirectedBreadthFirstIterator() = default;
    };

    template <typename VertexType, typename DirectedGraph>
    auto directedDepthFirst(const DirectedGraph& digraph, const VertexType& start,
        const visitedPtr_t<VertexType>& visited = std::make_shared<std::unordered_set<VertexType>>())
    {
        return std::make_pair(
            DirectedDepthFirstIterator<VertexType, DirectedGraph>(&digraph, start, visited),
            DirectedDepthFirstIterator<VertexType, DirectedGraph>());
    }

    template <typename VertexType, typename DirectedGraph>
    class DirectedDepthFirstIterator final: public boost::iterator_facade<
        DirectedDepthFirstIterator<VertexType, DirectedGraph>, const VertexType, std::input_iterator_tag>
    {
        friend class boost::iterator_core_access;
        friend auto directedDepthFirst<VertexType, DirectedGraph>(
            const DirectedGraph&, const VertexType&, const visitedPtr_t<VertexType>&);

        private:
            struct StackElem
            {
                VertexType vertex;
                typename return_t<decltype(&DirectedGraph::outNeighbors)(DirectedGraph, const VertexType&)>::type
                    outNeighborRange;
            };

            const DirectedGraph* m_graph = nullptr;
            VertexType m_start;
            std::shared_ptr<std::stack<StackElem>> m_vertexStack;
            visitedPtr_t<VertexType> m_visited;
            std::size_t m_poppedCount = 0;

            DirectedDepthFirstIterator(const DirectedGraph* digraph, const VertexType& start,
                const visitedPtr_t<VertexType>& visited):
                m_graph(digraph), m_start(start), m_vertexStack(new std::stack<StackElem>),
                m_visited(visited)
            {
                m_vertexStack->push({start, digraph->outNeighbors(start)});
                m_visited->insert(start);
                findDeadEnd();
            }

            const VertexType& dereference() const {return m_vertexStack->top().vertex;}

            void findDeadEnd()
            {
                bool hasUnvisitedChildren;
                do
                {
                    auto& range = m_vertexStack->top().outNeighborRange;
                    hasUnvisitedChildren = false;
                    for (; range.first != range.second; ++range.first)
                    {
                        if (!m_visited->count(*range.first))
                        {
                            m_vertexStack->push({*range.first, m_graph->outNeighbors(*range.first)});
                            m_visited->insert(*range.first);
                            hasUnvisitedChildren = true;
                        }
                    }
                }
                while (hasUnvisitedChildren);
            }

            bool equal(const DirectedDepthFirstIterator<VertexType, DirectedGraph>& other) const
            {
                bool otherStackIsEmpty = !other.m_vertexStack || !other.m_vertexStack->size();
                if (!m_vertexStack || !m_vertexStack->size())
                    return otherStackIsEmpty;
                else if (otherStackIsEmpty)
                    return false;
                // If they start with the same vertex and have traversed the same number of vertices, they are equal.
                else
                    return m_graph == other.m_graph && m_start == other.m_start && m_poppedCount == other.m_poppedCount;
            }

            void increment()
            {
                m_vertexStack->pop();
                ++m_poppedCount;
                if (m_vertexStack->size())
                    findDeadEnd();
            }

        public:
            DirectedDepthFirstIterator() = default;
    };
}

#endif
