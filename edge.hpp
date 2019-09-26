#ifndef GRAPH_EDGE_HPP_992019
#define GRAPH_EDGE_HPP_992019

#include <utility>
#include <type_traits>
#include <cstddef>
#include <functional>

namespace graph
{
    template <typename VertexType>
    class Edge
    {
        protected:
            VertexType m_tail, m_head;

        public:
            using vertex_t = VertexType;

            template <typename T>
            Edge(T&& tail, T&& head): m_tail(std::forward<T>(tail)), m_head(std::forward<T>(head)) {}

            Edge(const Edge<VertexType>& other) = default;
            Edge(Edge<VertexType>&& other): m_tail(std::move(other.m_tail)), m_head(std::move(other.m_head)) {}

            virtual ~Edge() {}

            const VertexType& tail() const {return m_tail;}
            const VertexType& head() const {return m_head;}

            Edge<VertexType>& operator=(const Edge<VertexType>& other) = default;

            Edge<VertexType>& operator=(Edge<VertexType>&& other)
            {
                if (&other != this)
                    m_tail = std::move(other.m_tail), m_head = std::move(other.m_head);
                return *this;
            }
    };

    template <typename VertexType>
    bool operator==(const Edge<VertexType>& first, const Edge<VertexType>& second)
    {
        return first.tail() == second.tail() && first.head() == second.head();
    }

    template <typename VertexType>
    bool operator!=(const Edge<VertexType>& first, const Edge<VertexType>& second) {return !(first == second);}

    template <typename VertexType, typename WeightType>
    class WeightedEdge: public Edge<VertexType>
    {
        static_assert(std::is_arithmetic<WeightType>::value, "'WeightType' must be an arithmetic type.");

        protected:
            WeightType m_weight;

        public:
            template <typename T>
            WeightedEdge(T&& tail, T&& head, WeightType weight): Edge<VertexType>(tail, head), m_weight(weight) {}

            WeightedEdge(const WeightedEdge<VertexType, WeightType>& other) = default;
            WeightedEdge(WeightedEdge<VertexType, WeightType>&& other):
                Edge<VertexType>(other), m_weight(other.m_weight) {}

            virtual ~WeightedEdge() {}

            WeightType weight() const {return m_weight;}

            template <typename T>
            void weight(T newWeight)
            {
                static_assert(std::is_arithmetic<T>::value, "'T' must be an arithmetic type.");
                m_weight = static_cast<WeightType>(newWeight);
            }

            WeightedEdge<VertexType, WeightType>& operator=(
                const WeightedEdge<VertexType, WeightType>& other) = default;

            WeightedEdge<VertexType, WeightType>& operator=(WeightedEdge<VertexType, WeightType>&& other)
            {
                Edge<VertexType>::operator=(other);
                m_weight = other.m_weight;
                return *this;
            }
    };
}

namespace std
{
    template <typename VertexType>
    struct hash<graph::Edge<VertexType>>
    {
        size_t operator()(const graph::Edge<VertexType>& edge) const
        {
            hash<VertexType> vertexHash;
            return vertexHash(edge.tail()) ^ (vertexHash(edge.head()) << 1);
        }
    };
}

#endif
