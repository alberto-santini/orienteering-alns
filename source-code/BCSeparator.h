//
// Created by alberto on 25/10/17.
//

#ifndef OP_BCSEPARATOR_H
#define OP_BCSEPARATOR_H

#include "Graph.h"

#include <memory>
#include <type_traits>
#include <as/graph.h>
#include <as/containers.h>

#define IL_STD
#include <ilcplex/ilocplex.h>

namespace op {
    template<class CPXCallback>
    class BCSeparator : public CPXCallback {
        // This class can inherit only from the two cplex cuts callbacks.
        static_assert(
            std::is_same<CPXCallback, IloCplex::LazyConstraintCallbackI>::value ||
            std::is_same<CPXCallback, IloCplex::UserCutCallbackI>::value,
            "CPXCallBack can only be IloCplex::LazyConstraintCallbackI or IloCplex::UserCutCallbackI"
        );

        /**
         * The underlying graph.
         */
        const Graph& graph;

        /**
         * The cplex environment.
         */
        const IloEnv& env;

        /**
         * Cplex arc variables.
         */
        const IloNumVarArray& x;

        /**
         * Cplex vertex variables.
         */
        const IloNumVarArray& y;

    public:

        /**
         * Creates a new separator.
         */
        BCSeparator(
            const Graph& graph,
            const IloEnv& env,
            const IloNumVarArray& x,
            const IloNumVarArray& y
        ) :
            CPXCallback{env},
            graph{graph},
            env{env},
            x{x},
            y{y}
        {}

        /**
         * Cloning method, required by cplex.
         */
        IloCplex::CallbackI* duplicateCallback() const override {
            return new(env) BCSeparator(graph, env, x, y);
        }

        /**
         * Gets the connected component starting at a certain vertex.
         */
        std::vector<BoostVertex> get_connected_component(BoostVertex starting_v) {
            using as::containers::contains;

            std::vector<BoostVertex> cc;
            BoostVertex current_v = starting_v;

            do {
                cc.push_back(current_v);

                const BoostVertex prev_v = current_v;

                for(const auto& next_v : as::graph::neighbours(current_v, graph.g)) {
                    if(contains(cc, next_v)) { continue; }

                    const auto edge = boost::edge(current_v, next_v, graph.g).first;
                    const auto edge_id = graph.g[edge].id;

                    if(this->template getValue(x[edge_id]) > .5) {
                        assert(this->template getValue(y[next_v]) > .5);

                        current_v = next_v;
                        break;
                    }
                }

                if(prev_v == current_v) {
                    assert(this->template getValue(x[graph.g[boost::edge(current_v, starting_v, graph.g).first].id]) > .5);
                    current_v = starting_v;
                }
            } while(current_v != starting_v);

            return cc;
        }

        /**
         * Main method, required by cplex.
         */
        void main() override {
            assert(this->template getValue(y[0u]) > .5);

            // First step: determine the connected component
            // which contains the depot.
            std::vector<BoostVertex> depot_cc = get_connected_component(0u);

            // THIS PRODUCES A B&C ALGORITHM WHICH IS AN EXACT METHOD:

            // Second step: get all other connected components.
            std::vector<std::vector<BoostVertex>> other_ccs;
            std::vector<BoostVertex> free = as::graph::vertex_complement(depot_cc, graph.g);

            while(!free.empty()) {
                auto free_vertex = free.back();

                if(this->template getValue(y[free_vertex]) < .5) {
                    free.pop_back();
                    continue;
                }

                other_ccs.push_back(get_connected_component(free_vertex));

                for(const auto& v : other_ccs.back()) {
                    free.erase(std::remove(free.begin(), free.end(), v), free.end());
                }
            }

            // Third step: for each connected component, break it
            // by specifying the number of internal edges which
            // can be selected.
            for(const auto& cc : other_ccs) {
                IloExpr expr = x[cc.front()];

                for(auto i = 1u; i < cc.size(); ++i) {
                    expr += x[cc[i]];
                }

                this->template add(expr <= static_cast<IloNum>(cc.size() - 1u));
            }

            // ------------------------------------------------------------------------------------

            // THIS PRODUCES A B&C ALGORITHM WHICH IS A HEURISTIC:

            // Second step: find all visited vertices which are
            // not in the connected component we just found, and
            // add a cut imposing flow between the connected
            // component and each vertex (if it is visited).

//            for(const auto& vertex : as::graph::vertices(graph.g)) {
//                if(this->template getValue(y[vertex]) < .5) { continue; }
//                if(as::contains(depot_cc, vertex)) { continue; }
//
//                IloExpr expr = -2 * y[vertex];
//
//                for(const auto& cc_vertex : depot_cc) {
//                    const auto edge = boost::edge(cc_vertex, vertex, graph.g).first;
//                    const auto edge_id = graph.g[edge].id;
//
//                    expr += x[edge_id];
//                }
//
//                this->template add(expr >= 0);
//            }
        }
    };
}

#endif //OP_BCSEPARATOR_H
