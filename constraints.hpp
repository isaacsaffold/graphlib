#ifndef GRAPH_CONSTRAINTS_HPP_9292019
#define GRAPH_CONSTRAINTS_HPP_9292019

namespace graph
{
    // conforms to `BitmaskType`
    enum class Constraints: unsigned int
    {
        NONE = 0,
        NO_LOOPS = 1,
        NO_MULTIPLE_EDGES = 2,
        SIMPLE = 3
    };

    constexpr Constraints operator&(Constraints a, Constraints b)
    {
        return static_cast<Constraints>(static_cast<unsigned int>(a) & static_cast<unsigned int>(b));
    }

    constexpr Constraints operator|(Constraints a, Constraints b)
    {
        return static_cast<Constraints>(static_cast<unsigned int>(a) | static_cast<unsigned int>(b));
    }

    constexpr Constraints operator^(Constraints a, Constraints b)
    {
        return static_cast<Constraints>(static_cast<unsigned int>(a) ^ static_cast<unsigned int>(b));
    }

    constexpr Constraints operator~(Constraints a) {return static_cast<Constraints>(~static_cast<unsigned int>(a));}

    constexpr bool operator==(Constraints a, unsigned int b) {return static_cast<unsigned int>(a) == b;}
    constexpr bool operator==(unsigned int a, Constraints b) {return a == static_cast<unsigned int>(b);}
    constexpr bool operator!=(Constraints a, unsigned int b) {return !(a == b);}
    constexpr bool operator!=(unsigned int a, Constraints b) {return !(a == b);}

    Constraints& operator&=(Constraints& a, Constraints b) {return a = a & b;}
    Constraints& operator|=(Constraints& a, Constraints b) {return a = a | b;}
    Constraints& operator^=(Constraints& a, Constraints b) {return a = a ^ b;}
}

#endif
