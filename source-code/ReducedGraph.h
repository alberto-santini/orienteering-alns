//
// Created by alberto on 22/10/17.
//

#ifndef OP_REDUCEDGRAPH_H
#define OP_REDUCEDGRAPH_H

#include "Graph.h"
#include "Clustering.h"
#include <experimental/memory>
#include <map>
#include <optional>

namespace op {
    struct ReducedGraph {
        /**
         * The original graph, before reduction.
         */
        std::experimental::observer_ptr<const Graph> original_graph;

        /**
         * The reduced graph.
         */
        Graph reduced_graph;

        /**
         * A mapping of the new vertex numbering (in the reduced graph)
         * to the original one (in the original graph).
         */
        std::map<BoostVertex, std::vector<BoostVertex>> vertices_mapping;

        /**
         * TSPs for those points which are mapped to > 1 vertex on the
         * original graph.
         */
        std::map<BoostVertex, Tour> tsps;

        /**
         * Empty constructor.
         */
        ReducedGraph() = default;

        /**
         * Builds the reduced graph from an (original) graph.
         *
         * @param original_graph The original graph.
         */
        explicit ReducedGraph(const Graph *const original_graph);

        /**
         * Builds the reduced graph from an (original) graph, when
         * the clustering for the original graph has already been
         * computed.
         *
         * @param original_graph    The original graph.
         * @param c                 A clustering for the original graph.
         */
        ReducedGraph(const Graph *const original_graph, const Clustering& c);

        /**
         * Applies clustering reduction recursively on a given Graph.
         * It stops when no proper clustering is possible, or the number
         * of vertices in the graph has been reduced by a factor of at
         * least red_factor. If no proper clustering is possible already
         * for the original graph, it returns std::nullopt.
         *
         * @param graph The original graph.
         */
        friend std::optional<ReducedGraph> recursive_reduction(const Graph *const graph, float red_factor);

        /**
         * Takes a tour on the reduced graph and builds a tour on the
         * original graph. The cluster tsps will begin at the node which
         * minimises the distance to the previous and next nodes (or
         * centres of mass). If the resulting tour is too long, then
         * vertices are removed to make it shorter. The vertices are
         * removed according to the ratio p/g where p is the profit and
         * g is the gain in travel time obtained by removing the vertex.
         * We remove first the vertices with the smallest p/g.
         *
         * @param tour  Tour on the reduced graph.
         * @param red   Reduced graph.
         * @return      Tour on the original graph.
         */
        friend Tour project_back_tour(const Tour& tour, const ReducedGraph& red);

        /**
         * Reduces again an already reduced graph.
         *
         * @param other The reduced graph to reduce again.
         * @return      The re-reduced graph.
         */
        friend ReducedGraph reduce_again(const ReducedGraph& other);

        /**
         * Reduces again an already reduced graph, when a
         * clustering for the already-reduced graph is available.
         *
         * @param other The reduced graph to reduce again.
         * @param c     A clustering for the reduced graph.
         * @return      The re-reduced graph.
         */
        friend ReducedGraph reduce_again(const ReducedGraph& other, const Clustering& c);
    };

    std::optional<ReducedGraph> recursive_reduction(const Graph *const graph, float red_factor = 0.5f);
    Tour project_back_tour(const Tour& tour, const ReducedGraph& red);
    ReducedGraph reduce_again(const ReducedGraph& other);
    ReducedGraph reduce_again(const ReducedGraph& other, const Clustering& c);
}

#endif //OP_REDUCEDGRAPH_H
