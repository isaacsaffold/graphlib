#ifndef GRAPH_ALGORITHMS_HPP_9192019
#define GRAPH_ALGORITHMS_HPP_9192019

#include <queue>
#include <stack>
#include <iterator>
#include <unordered_set>
#include <utility>
#include <cstddef>
#include <boost/iterator/iterator_facade.hpp>

namespace graph
{
    template <typename VertexType, typename EdgeType, template <typename, typename> typename DirectedGraph>
    class DirectedBreadthFirstIterator final: public boost::iterator_facade<
        DirectedBreadthFirstIterator<VertexType, EdgeType, DirectedGraph>, const VertexType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            const DirectedGraph<VertexType, EdgeType>* m_graph = nullptr;
            std::queue<VertexType> m_vertexQueue;
            std::unordered_set<VertexType> m_visited;
            VertexType m_start;
            std::size_t m_poppedCount = 0;

            const VertexType& dereference() const {return m_vertexQueue.front();}

            bool equal(const DirectedBreadthFirstIterator<VertexType, EdgeType, DirectedGraph>& other) const
            {
                if (!m_vertexQueue.size())
                    return !other.m_vertexQueue.size();
                else if (!other.m_vertexQueue.size())
                    return false;
                // If they start with the same vertex and have traversed the same number of vertices, they are equal.
                else
                    return m_graph == other.m_graph && m_start == other.m_start && m_poppedCount == other.m_poppedCount;
            }

            void increment()
            {
                auto iters(m_graph->outNeighbors(m_vertexQueue.front()));
                for (; iters.first != iters.second; ++iters.first)
                {
                    if (!m_visited.count(*iters.first))
                    {
                        m_vertexQueue.push(*iters.first);
                        m_visited.insert(*iters.first);
                    }
                }
                m_vertexQueue.pop();
                ++m_poppedCount;
            }

        public:
            DirectedBreadthFirstIterator() = default;
            DirectedBreadthFirstIterator(const DirectedGraph<VertexType, EdgeType>* digraph, const VertexType& start):
                m_graph(digraph), m_start(start)
            {
                m_vertexQueue.push(start);
                m_visited.insert(start);
            }
    };

    template <typename VertexType, typename EdgeType, template <typename, typename> typename DirectedGraph>
    class DirectedDepthFirstIterator final: public boost::iterator_facade<
        DirectedDepthFirstIterator<VertexType, EdgeType, DirectedGraph>, const VertexType, std::forward_iterator_tag>
    {
        friend class boost::iterator_core_access;

        private:
            const DirectedGraph<VertexType, EdgeType>* m_graph = nullptr;
            std::stack<VertexType> m_vertexStack;
            std::unordered_set<VertexType> m_visited;
            VertexType m_start;
            std::size_t m_poppedCount = 0;

            const VertexType& dereference() const {return m_vertexStack.top();}

            bool equal(const DirectedDepthFirstIterator<VertexType, EdgeType, DirectedGraph>& other) const
            {
                if (!m_vertexStack.size())
                    return !other.m_vertexStack.size();
                else if (!other.m_vertexStack.size())
                    return false;
                // If they start with the same vertex and have traversed the same number of vertices, they are equal.
                else
                    return m_graph == other.m_graph && m_start == other.m_start && m_poppedCount == other.m_poppedCount;
            }

            void increment()
            {
                VertexType vertex(m_vertexStack.top());
                m_vertexStack.pop();
                ++m_poppedCount;
                for (auto iters(m_graph->outNeighbors(vertex)); iters.first != iters.second; ++iters.first)
                {
                    if (!m_visited.count(*iters.first))
                    {
                        m_vertexStack.push(*iters.first);
                        m_visited.insert(*iters.first);
                    }
                }
            }

        public:
            DirectedDepthFirstIterator() = default;
            DirectedDepthFirstIterator(const DirectedGraph<VertexType, EdgeType>* digraph, const VertexType& start):
                m_graph(digraph), m_start(start)
            {
                m_vertexStack.push(start);
                m_visited.insert(start);
            }
    };

    // convenience function for creating a `DirectedBreadthFirstIterator`
    template <typename VertexType, typename EdgeType, template <typename, typename> typename DirectedGraph>
    auto directedBreadthFirst(const DirectedGraph<VertexType, EdgeType>& digraph, const VertexType& start)
    {
        return std::make_pair(
            DirectedBreadthFirstIterator<VertexType, EdgeType, DirectedGraph>(&digraph, start),
            DirectedBreadthFirstIterator<VertexType, EdgeType, DirectedGraph>());
    }

    // convenience function for creating a `DirectedDepthFirstIterator`
    template <typename VertexType, typename EdgeType, template <typename, typename> typename DirectedGraph>
    auto directedDepthFirst(const DirectedGraph<VertexType, EdgeType>& digraph, const VertexType& start)
    {
        return std::make_pair(
            DirectedDepthFirstIterator<VertexType, EdgeType, DirectedGraph>(&digraph, start),
            DirectedDepthFirstIterator<VertexType, EdgeType, DirectedGraph>());
    }
}

#endif
