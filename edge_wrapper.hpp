#ifndef GRAPH_EDGE_WRAPPER_HPP_1082019
#define GRAPH_EDGE_WRAPPER_HPP_1082019

#include <type_traits>

namespace graph
{
    template <typename Vertex>
    struct p_ImplicitEdgeWrapper
    {
        using vertex_t = Vertex;
        using edge_t = void;

        Vertex *tail, *head;

        p_ImplicitEdgeWrapper() = default;
        p_ImplicitEdgeWrapper(Vertex* tail, Vertex* head) {this->tail = tail, this->head = head;}

        template <typename T, std::enable_if_t<std::is_convertible<T*, Vertex*>::value, bool> = true>
        p_ImplicitEdgeWrapper(const p_ImplicitEdgeWrapper<T>& other):
            tail(other.tail), head(other.head) {}

        template <typename T, std::enable_if_t<std::is_convertible<T*, Vertex*>::value, bool> = true>
        p_ImplicitEdgeWrapper& operator=(p_ImplicitEdgeWrapper<T>&& other)
        {
            tail = other.tail, head = other.head;
            return *this;
        }
    };

    template <typename V1, typename V2>
    bool operator==(const p_ImplicitEdgeWrapper<V1>& first, const p_ImplicitEdgeWrapper<V2>& second)
    {
        return first.tail == second.tail && first.head == second.head;
    }

    template <typename V1, typename V2>
    bool operator!=(const p_ImplicitEdgeWrapper<V1>& first, const p_ImplicitEdgeWrapper<V2>& second)
    {
        return !(first == second);
    }

    template <typename Vertex, typename Edge>
    struct p_EdgeWrapper
    {
        using vertex_t = Vertex;
        using edge_t = Edge;

        Vertex *tail, *head;
        Edge* edge;

        p_EdgeWrapper() = default;
        p_EdgeWrapper(Vertex* tail, Vertex* head, Edge* edge) {this->tail = tail, this->head = head, this->edge = edge;}

        template <typename T, typename U, std::enable_if_t<
            std::is_convertible<T*, Vertex*>::value && std::is_convertible<U*, Edge*>::value, bool> = true>
        p_EdgeWrapper(const p_EdgeWrapper<T, U>& other): tail(other.tail), head(other.head), edge(other.edge) {}

        template <typename T, typename U, std::enable_if_t<
            std::is_convertible<T*, Vertex*>::value && std::is_convertible<U*, Edge*>::value, bool> = true>
        p_EdgeWrapper& operator=(const p_EdgeWrapper<T, U>& other)
        {
            tail = other.tail, head = other.head, edge = other.edge;
            return *this;
        }
    };

    template <typename V1, typename V2, typename E1, typename E2>
    bool operator==(const p_EdgeWrapper<V1, E1>& first, const p_EdgeWrapper<V2, E2>& second)
    {
        return first.tail == second.tail && first.head == second.head && first.edge == second.edge;
    }

    template <typename V1, typename V2, typename E1, typename E2>
    bool operator!=(const p_EdgeWrapper<V1, E1>& first, const p_EdgeWrapper<V2, E2>& second)
    {
        return !(first == second);
    }

    template <typename Vertex, typename Edge = void>
    using edgeWrapper_t =
        std::conditional_t<std::is_void<Edge>::value, p_ImplicitEdgeWrapper<Vertex>, p_EdgeWrapper<Vertex, Edge>>;
}

#endif
