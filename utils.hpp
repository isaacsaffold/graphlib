#ifndef GRAPH_UTILS_HPP_9122019
#define GRAPH_UTILS_HPP_9122019

namespace graph
{
    template <typename EdgeType>
    bool areParallel(const EdgeType& first, const EdgeType& second)
    {
        return first.tail() == second.tail() && first.head() == second.head();
    }

    template <typename EdgeType>
    bool haveSameEndpoints(const EdgeType& first, const EdgeType& second)
    {
        return areParallel(first, second) || (first.tail() == second.head() && first.head() == second.tail());
    }
}

#endif
