#ifndef GRAPH_EDGE_WRAPPER_HPP_1082019
#define GRAPH_EDGE_WRAPPER_HPP_1082019

#include <type_traits>

namespace graph
{
    template <typename Vertex>
    struct p_ImplicitEdgeWrapper
    {
        Vertex *tail, *head;
    };

    template <typename Vertex, typename Edge>
    struct p_EdgeWrapper
    {
        Vertex *tail, *head;
        Edge* edge;
    };

    template <typename Vertex, typename Edge = void>
    using edgeWrapper_t =
        std::conditional_t<std::is_void<Edge>::value, p_ImplicitEdgeWrapper<Vertex>, p_EdgeWrapper<Vertex, Edge>>;
}

#endif
