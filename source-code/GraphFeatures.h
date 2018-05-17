//
// Created by alberto on 12/02/18.
//

#ifndef OP_GRAPHFEATURES_H
#define OP_GRAPHFEATURES_H

#include "Graph.h"
#include "Clustering.h"

#include <ostream>
#include <experimental/memory>

namespace op {
    namespace features {
        float diameter(const Graph& graph);
        float max_distance_from_depot(const Graph& graph);
        float distance_btw_barycentre_and_depot(const Graph& graph);
        float graph_spread_frac(const Graph& graph);
        float largest_set_of_aligned_pts_frac(const Graph& graph);
        float number_of_clusters_frac(const Clustering& clustering);
        float avg_cluster_size_frac(const Clustering& clustering);
        float avg_cluster_diameter_frac(const Clustering& clustering);
        float cluster_spread_frac(const Clustering& clustering);
        float cluster_and_isolated_spread_frac(const Clustering& clustering);
        float isolated_vertices_frac(const Clustering& clustering);

        void print_features(std::ostream& out, const Graph& graph, std::experimental::observer_ptr<Clustering> clustering = nullptr);
    }
}

#endif //OP_GRAPHFEATURES_H
