//
// Created by alberto on 18/10/17.
//

#ifndef OP_RTREEUTILS_H
#define OP_RTREEUTILS_H

#include "Graph.h"

namespace op {
    /**
     * Returns a list of points which are within a certain radius from the centre.
     *
     * @param centre    The centre point.
     * @param radius    The radius.
     * @param t         The RTree on which to perform the search.
     * @return          A list of points within distance "radius" from "centre" (including itself).
     */
    std::vector<BoostTreeValue> within_radius(const BoostPoint& centre, float radius, const BoostRTree& r);

    /**
     * Return a list of points which are within a minimum and a maximum radius from the centre.
     *
     * @param centre    The centre point.
     * @param min_r     Min distance from the centre.
     * @param max_r     Max distance from the centre.
     * @param r         The RTree on which to perform the search.
     * @return          A list of points satisfying the requisites.s
     */
    std::vector<BoostTreeValue> within_radii(const BoostPoint& centre, float min_r, float max_r, const BoostRTree& r);

    /**
     * Runs the DBSCAN clustering algorithm on the graph.
     *
     * @param g         The graph.
     * @param radius    Neighbours of a vertex are searched within this radius.
     * @param min_pts   Minimum number of points to create a cluster.
     * @return          A clustering of the graph's vertices (not a partition, as some vertex might be left out).
     */
    std::vector<std::vector<BoostVertex>> dbscan(const Graph& g, float radius, std::size_t min_pts);

    /**
     * Runs the DBSCAN clustering algorithm with the radius and min_pts parameter obtained
     * using a "guessing" method.
     *
     * @param g The graph.
     * @return  A clustering of the graph's vertices (not a partition, as some vertex might be left out).
     */
    std::vector<std::vector<BoostVertex>> dbscan(const Graph& g);

    /**
     * Returns a vector containing the distances from each vertex of the
     * graph to its nearest neighbour. The depot and unreachable vertices
     * are ignored. The vector is returned sorted from small to large.
     *
     * @param g The graph.
     * @return  The distance vector.
     */
    std::vector<float> nearest_neighbour_distances(const Graph& g);

    /**
     * Returns a vector containing the size of the neighbourhoods centred
     * at each (non-depot, reachable) vertex with radius "radius". The
     * vector is returned sorted from small to large.
     *
     * @param radius    The neighbourhood-defining radius.
     * @param g         The graph.
     * @return          The size vector.
     */
    std::vector<std::size_t> neighbourhood_sizes(float radius, const Graph& g);
}

#endif //OP_RTREEUTILS_H
