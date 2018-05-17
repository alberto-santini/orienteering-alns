//
// Created by alberto on 12/05/18.
//

#ifndef OP_TOURREMOVALLABELLING_H
#define OP_TOURREMOVALLABELLING_H

#include <boost/property_map/property_map.hpp>
#include <type_traits>
#include "Graph.h"

namespace op {
    struct JVertexProperty {
        std::size_t id;
        BoostVertex original_v;
        float prize;
        JVertexProperty() = default;
        JVertexProperty(std::size_t i, BoostVertex o, float p) : id{i}, original_v{o}, prize{p} {}
    };

    struct JEdgeProperty {
        std::size_t id;
        float distance;
        JEdgeProperty() = default;
        JEdgeProperty(std::size_t i, float d) : id{i}, distance{d} {}
    };

    using JGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, JVertexProperty, JEdgeProperty, float>;
    using JVertex = boost::graph_traits<JGraph>::vertex_descriptor;
    using JEdge = boost::graph_traits<JGraph>::edge_descriptor;

    struct JLabel {
        float distance;
        float prize;

        JLabel(float d, float p) : distance{d}, prize{p} {}
        bool operator==(const JLabel& other) const {
            return std::fabs(distance - other.distance) < 1e-6f && std::fabs(prize - other.prize) < 1e-6f;
        }
        bool operator!=(const JLabel& other) const {
            return !(*this == other);
        }
        bool operator<(const JLabel& other) const {
            return (distance <= other.distance) && (prize <= other.prize) && (*this != other);
        }
    };

    struct JLabelExtender {
        bool operator()(const JGraph& jgraph, JLabel& new_label, const JLabel& label, const JEdge& edge) const {
            const auto& ttime = jgraph[edge].distance;

            if(label.distance + ttime > jgraph[boost::graph_bundle]) {
                return false;
            }

            const auto& dest = boost::target(edge, jgraph);

            new_label.distance = label.distance + ttime;
            new_label.prize = label.prize + jgraph[dest].prize;

            return true;
        }
    };

    struct JVFunctor {
        const JGraph& jgraph;
        JVFunctor(const JGraph& j) : jgraph{j} {}
        using result_type = std::size_t;
        result_type operator()(const JVertex& v) const {
            return jgraph[v].id;
        }
    };

    struct JEFunctor {
        const JGraph& jgraph;
        JEFunctor(const JGraph& j) : jgraph{j} {}
        using result_type = std::size_t;
        result_type operator()(const JEdge& e) const {
            return jgraph[e].id;
        }
    };

    template<class Fn, class Arg>
    struct FnPM {
        const Fn& fn;
        using value_type = typename std::result_of<Fn(Arg)>::type;
        FnPM(const Fn& fn) : fn{fn} {}
        friend value_type get(const FnPM& pm, const Arg& arg) { return pm.fn(arg); }
        value_type operator[](const Arg& arg) const { return fn(arg); }
    };

    template<typename Arg, typename Fn>
    FnPM<Fn, Arg> make_property_map(const Fn &f) {
        return FnPM<Fn, Arg>(f);
    }
}

namespace boost {
    template<typename Fn, typename Arg>
    struct property_traits<op::FnPM<Fn, Arg>> {
        using value_type = typename std::result_of<Fn(Arg)>::type;
        using reference = value_type;
        using key_type = Arg;
        using category = readable_property_map_tag;
    };
}


#endif //OP_TOURREMOVALLABELLING_H
