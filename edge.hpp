#ifndef GRAPH_EDGE_HPP_992019
#define GRAPH_EDGE_HPP_992019

#include <utility>
#include <cstddef>
#include <functional>
#include <boost/functional/hash.hpp>

#include "utils.hpp"

namespace graph
{
    template <typename VertexType, bool directed>
    class Edge
    {
        protected:
            VertexType m_tail, m_head;

        public:
            using vertex_t = VertexType;

            static constexpr bool isDirected() {return directed;}

            Edge() = default;

            template <typename T>
            Edge(T&& tail, T&& head): m_tail(std::forward<T>(tail)), m_head(std::forward<T>(head)) {}

            Edge(const Edge<VertexType, directed>& edge) = default;
            Edge(Edge<VertexType, directed>&& edge) noexcept = default;

            virtual ~Edge() = default;

            const VertexType& tail() const {return m_tail;}
            const VertexType& head() const {return m_head;}

            Edge<VertexType, directed>& operator=(const Edge<VertexType, directed>& other) = default;
            Edge<VertexType, directed>& operator=(Edge<VertexType, directed>&& other) noexcept = default;

            explicit operator Edge<VertexType, !directed>() const
            {
                return Edge<VertexType, !directed>(m_tail, m_head);
            }
    };

    template <typename VertexType>
    bool operator==(const Edge<VertexType, true>& first, const Edge<VertexType, true>& second)
    {
        return haveSameEndpoints(first, second);
    }

    template <typename VertexType>
    bool operator==(const Edge<VertexType, false>& first, const Edge<VertexType, false>& second)
    {
        return areParallel(first, second);
    }

    template <typename VertexType, bool directed>
    bool operator!=(const Edge<VertexType, directed>& first, const Edge<VertexType, directed>& second)
    {
        return !(first == second);
    }

    template <typename VertexType, bool directed, typename WeightType>
    class WeightedEdge: public Edge<VertexType, directed>
    {
        protected:
            using Edge<VertexType, directed>::m_tail;
            using Edge<VertexType, directed>::m_head;

            WeightType m_weight;

        public:
            using weight_t = WeightType;

            WeightedEdge() = default;

            template <typename T>
            WeightedEdge(T&& tail, T&& head, WeightType weight = static_cast<WeightType>(1)):
                Edge<VertexType, directed>(tail, head), m_weight(weight) {}

            WeightedEdge(const WeightedEdge<VertexType, directed, WeightType>& other) = default;
            WeightedEdge(WeightedEdge<VertexType, directed, WeightType>&& other) noexcept = default;

            WeightedEdge(const Edge<VertexType, directed>& edge): WeightedEdge(edge.m_tail, edge.m_head) {}
            WeightedEdge(Edge<VertexType, directed>&& edge):
                WeightedEdge(std::move(edge.m_tail), std::move(edge.m_head)) {}

            virtual ~WeightedEdge() = default;

            WeightType weight() const {return m_weight;}

            template <typename OtherWeightType>
            void weight(OtherWeightType newWeight)
            {
                m_weight = static_cast<WeightType>(newWeight);
            }

            WeightedEdge<VertexType, directed, WeightType>& operator=(
                const WeightedEdge<VertexType, directed, WeightType>& other) = default;
            WeightedEdge<VertexType, directed, WeightType>& operator=(
                WeightedEdge<VertexType, directed, WeightType>&& other) noexcept = default;

            WeightedEdge<VertexType, directed, WeightType>& operator=(const Edge<VertexType, directed>& other)
            {
                if (&other != this)
                    m_tail = other.m_tail, m_head = other.m_head, m_weight = other.m_weight;
                return *this;
            }

            WeightedEdge<VertexType, directed, WeightType>& operator=(Edge<VertexType, directed>&& other)
            {
                if (&other != this)
                    m_tail = std::move(other.m_tail), m_head = std::move(other.m_head), m_weight = other.m_weight;
                return *this;
            }

            template <typename OtherWeightType>
            operator WeightedEdge<VertexType, directed, OtherWeightType>() const
            {
                return WeightedEdge<VertexType, directed, OtherWeightType>(m_tail, m_head);
            }

            template <typename OtherWeightType>
            explicit operator WeightedEdge<VertexType, !directed, OtherWeightType>() const
            {
                return WeightedEdge<VertexType, !directed, OtherWeightType>(m_tail, m_head);
            }
    };
}

namespace std
{
    template <typename VertexType>
    struct hash<graph::Edge<VertexType, true>>
    {
        size_t operator()(const graph::Edge<VertexType, true>& edge) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, edge.tail());
            boost::hash_combine(seed, edge.head());
            return seed;
        }
    };

    template <typename VertexType>
    struct hash<graph::Edge<VertexType, false>>
    {
        size_t operator()(const graph::Edge<VertexType, false>& edge) const
        {
            hash<VertexType> hasher;
            return hasher(edge.tail()) ^ hasher(edge.head());
        }
    };
}

#endif
