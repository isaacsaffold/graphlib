#ifndef GRAPH_CONSTRAINTS_HPP_9292019
#define GRAPH_CONSTRAINTS_HPP_9292019

#include <stdexcept>

namespace graph
{
    // conforms to `BitmaskType`
    enum class Constraints: unsigned char
    {
        NONE = 0,
        NO_LOOPS = 1,
        NO_MULTIPLE_EDGES = 2,
        SIMPLE = 3,
        CONNECTED = 4,
        CONNECTED_AND_NO_LOOPS = 5,
        CONNECTED_AND_NO_MULTIPLE_EDGES = 6,
        SIMPLY_CONNECTED = 7
    };

    constexpr Constraints operator&(Constraints a, Constraints b)
    {
        return static_cast<Constraints>(static_cast<unsigned char>(a) & static_cast<unsigned char>(b));
    }

    constexpr Constraints operator|(Constraints a, Constraints b)
    {
        return static_cast<Constraints>(static_cast<unsigned char>(a) | static_cast<unsigned char>(b));
    }

    constexpr Constraints operator^(Constraints a, Constraints b)
    {
        return static_cast<Constraints>(static_cast<unsigned char>(a) ^ static_cast<unsigned char>(b));
    }

    constexpr Constraints operator~(Constraints a) {return static_cast<Constraints>(~static_cast<unsigned char>(a));}

    constexpr bool operator==(Constraints a, unsigned char b) {return static_cast<unsigned char>(a) == b;}
    constexpr bool operator==(unsigned char a, Constraints b) {return a == static_cast<unsigned char>(b);}
    constexpr bool operator!=(Constraints a, unsigned char b) {return !(a == b);}
    constexpr bool operator!=(unsigned char a, Constraints b) {return !(a == b);}

    Constraints& operator&=(Constraints& a, Constraints b) {return a = a & b;}
    Constraints& operator|=(Constraints& a, Constraints b) {return a = a | b;}
    Constraints& operator^=(Constraints& a, Constraints b) {return a = a ^ b;}

    class ConstraintViolationException: public std::runtime_error
    {
        private:
            static const char* exceptionString(Constraints constraints)
            {
                switch (constraints)
                {
                    case Constraints::NO_LOOPS:
                        return "Graph cannot contain loops.";
                    case Constraints::NO_MULTIPLE_EDGES:
                        return "Graph cannot contain multiple edges.";
                    case Constraints::SIMPLE:
                        return "Graph must be simple (i.e. must not contain loops or multiple edges).";
                    case Constraints::CONNECTED:
                        return "Graph must be connected.";
                    case Constraints::CONNECTED_AND_NO_LOOPS:
                        return "Graph must be connected and cannot contain loops.";
                    case Constraints::CONNECTED_AND_NO_MULTIPLE_EDGES:
                        return "Graph must be connected and cannot contain multiple edges.";
                    case Constraints::SIMPLY_CONNECTED:
                        return "Graph must be connected and simple (i.e. must not contain loops or multiple edges).";
                    default:
                        return "";
                }
            }

        public:
            using std::runtime_error::runtime_error;

            explicit ConstraintViolationException(Constraints constraints) noexcept:
                std::runtime_error(exceptionString(constraints)) {}

            virtual ~ConstraintViolationException() = default;
    };
}

#endif
