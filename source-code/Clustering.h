//
// Created by alberto on 21/10/17.
//

#ifndef OP_CLUSTERING_H
#define OP_CLUSTERING_H

#include "Tour.h"
#include <vector>

namespace op {
    // Forward-declaration.
    class Graph;

    /**
     * A clustering of the vertices, i.e. a set of subsets V which are
     * mutually disjoint (but whose union does not necessarily cover V.
     */
    using VertexClustering = std::vector<std::vector<BoostVertex>>;

    /**
     * This class represents a clustering of (some of) the
     * vertices of the graph.
     */
    struct Clustering {
        /**
         * Underlying graph pointer.
         */
        std::experimental::observer_ptr<const Graph> graph;

        /**
         * The actual clustering.
         */
        VertexClustering clusters;

        /**
         * Number of clusters.
         */
        std::size_t n_clusters;

        /**
         * The centre of mass of each cluster.
         */
        std::vector<BoostPoint> centres;

        /**
         * The total prizes that could be collected
         * at each cluster, if all its vertices were
         * visited.
         */
        std::vector<float> prizes;

        /**
         * Vertices which do not belong to any cluster, besides
         * the depot and the unreachable vertices.
         */
        std::vector<BoostVertex> noise;

        /**
         * Empty constructor.
         */
        Clustering() = default;

        /**
         * Compute the clustering for a graph.
         *
         * @param graph The underlying graph.
         */
        explicit Clustering(const Graph *const graph);

        /**
         * A clustering is a proper clustering if any of the
         * following two prepositions is true:
         * - It has 2 <= n_clusters <= |V| - 1;
         * - It has n_clusters == 1 and 2 <= |clusters[0]| <= |V| - 1.
         *
         * @return Whether or not the clustering is proper.
         */
        bool is_proper() const;

    private:

        /**
         * Calculate which vertices are noise.
         */
        void calculate_noise();

        /**
         * Calculate statistics, such as centres and prizes.
         */
        void calculate_stats();
    };
}

#endif //OP_CLUSTERING_H
