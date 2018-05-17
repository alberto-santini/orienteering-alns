//
// Created by alberto on 24/10/17.
//

#include "BCSolver.h"
#include "BCSeparator.h"

#define IL_STD
#include <ilcplex/ilocplex.h>
#include <as/graph.h>
#include <as/console.h>
#include <as/and_die.h>

namespace op {
    Tour BCSolver::solve() const {
        using namespace as::console;
        using namespace as::graph;
        using as::containers::contains;
        using as::and_die;

        IloEnv env;
        IloModel model(env);

        IloNumVarArray x(env, boost::num_edges(graph.g));
        IloNumVarArray y(env, boost::num_vertices(graph.g));

        IloExpr expr(env);

        for(const auto& edge : edges(graph.g)) {
            const auto edge_id = graph.g[edge].id;

            assert(static_cast<long int>(edge_id) < x.getSize());

            x[edge_id] = IloNumVar(env, 0, 1, IloNumVar::Bool);

            expr += static_cast<double>(graph.g[edge].travel_time) * x[edge_id];
        }

        model.add(IloRange(env, 0, expr, graph.max_travel_time));
        expr.clear();

        for(const auto& vertex : vertices(graph.g)) {
            const auto vertex_id = graph.g[vertex].id;
            const double lower_bound = (vertex_id == 0u) ? 1.0 : 0.0;
            const double upper_bound = (graph.g[vertex].reachable ? 1.0 : 0.0);

            assert(static_cast<long int>(vertex_id) < y.getSize());

            y[vertex_id] = IloNumVar(env, lower_bound, upper_bound, IloNumVar::Bool);

            expr += static_cast<double>(graph.g[vertex].prize) * y[vertex_id];
        }

        model.add(IloObjective(env, expr, IloObjective::Maximize));
        expr.clear();

        for(const auto& vertex : vertices(graph.g)) {
            const auto vertex_id = graph.g[vertex].id;

            assert(static_cast<long int>(vertex_id) < y.getSize());

            for(const auto& edge : out_edges(vertex, graph.g)) {
                const auto edge_id = graph.g[edge].id;

                assert(static_cast<long int>(edge_id) < x.getSize());

                expr += x[edge_id];
                model.add(IloRange(env, -IloInfinity, x[edge_id] - y[vertex_id], 0));
            }

            expr -= 2 * y[vertex_id];
            model.add(IloRange(env, 0, expr, 0));
            expr.clear();
        }

        IloCplex cplex(model);

        std::cout << notice << "Running cplex on a graph with " << graph.n_vertices << " vertices." << std::endl;

        cplex.setOut(env.getNullStream());
        cplex.setError(env.getNullStream());
        cplex.use(IloCplex::Callback(new(env) BCSeparator<IloCplex::LazyConstraintCallbackI>(graph, env, x, y)));

        Tour solution;
        std::vector<BoostVertex> sol_vertices = {0u};
        std::vector<BoostEdge> sol_edges;

        try {
            if(cplex.solve()) {
                auto current_vertex = 0u;

                do {
                    // Using the proximity map, as it is much more likely for a vertex
                    // to be connected with another vertex which is close, rather than
                    // to a vertex which is fara way from it.

                    const auto prev_vertex = current_vertex;

                    for(const auto& vertex : as::graph::neighbours(current_vertex, graph.g)) {
                        // Do not go back on the same vertex you came from.
                        if(contains(sol_vertices, vertex)) { continue; }

                        const auto edge = boost::edge(current_vertex, vertex, graph.g).first;
                        const auto edge_id = graph.g[edge].id;
                        if(cplex.getValue(x[edge_id]) > 0) {
                            sol_vertices.push_back(vertex);
                            sol_edges.push_back(edge);
                            current_vertex = vertex;
                        }
                    }

                    if(prev_vertex == current_vertex) {
                        auto back_edge = boost::edge(current_vertex, 0u, graph.g).first;

                        assert(cplex.getValue(x[graph.g[back_edge].id]));

                        sol_edges.push_back(back_edge);
                        current_vertex = 0u;
                    }
                } while(current_vertex != 0u);

                solution = Tour(&graph, sol_edges);
                assert(solution.is_travel_time_correct());
            } else {
                std::cerr << error << "Cplex could not solve the problem!" << and_die();
            }
        } catch(IloException& e) {
            std::cerr << error << "Cplex exception: " << e.getMessage() << and_die();
        }

        env.end();

        return solution;
    }
}