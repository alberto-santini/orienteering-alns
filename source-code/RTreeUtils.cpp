//
// Created by alberto on 18/10/17.
//

#include "RTreeUtils.h"
#include <as/containers.h>
#include <as/console.h>
#include <as/graph.h>
#include <as/and_die.h>

namespace op {
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    std::vector<BoostTreeValue> within_radius(const BoostPoint& centre, float radius, const BoostRTree& r) {
        std::vector<BoostTreeValue> results;

        // If we simply listed all points and checked their distance from the centre, we would have
        // an O(n) alforithm. But we can do better: we take the rectangle (BoostBox) which inscribes
        // the circle centred in "centre" and of radius "radius". In the RTree, we can check whether
        // points are in this box in O(log n) iterations. Only for these points, we perform the
        // distance check.

        // Box inscribing the circle.
        const BoostBox bounding(
            BoostPoint(centre.x() - radius, centre.y() - radius),
            BoostPoint(centre.x() + radius, centre.y() + radius)
        );

        r.query(
            bgi::within(bounding) &&
            bgi::satisfies([&] (const BoostTreeValue& value) { return bg::distance(centre, value.first) <= radius; }) &&
            bgi::satisfies([] (const BoostTreeValue& value) { return value.second != 0u; }), // Exclude the depot
            std::back_inserter(results)
        );

        return results;
    }

    std::vector<BoostTreeValue> within_radii(const BoostPoint& centre, float min_r, float max_r, const BoostRTree& r) {
        std::vector<BoostTreeValue> results;

        float side = min_r / M_SQRT2;

        const BoostBox inner(
            BoostPoint(centre.x() - side, centre.y() - side),
            BoostPoint(centre.x() + side, centre.y() + side)
        );

        const BoostBox outer(
            BoostPoint(centre.x() - max_r, centre.y() - max_r),
            BoostPoint(centre.x() + max_r, centre.y() + max_r)
        );

        r.query(
            bgi::within(outer) &&
            bgi::disjoint(inner) &&
            bgi::satisfies([&] (const BoostTreeValue& value) { return bg::distance(centre, value.first) <= max_r; }) &&
            bgi::satisfies([&] (const BoostTreeValue& value) { return bg::distance(centre, value.first) >= min_r; }) &&
            bgi::satisfies([] (const BoostTreeValue& value) { return value.second != 0u; }), // Exclude the depot
            std::back_inserter(results)
        );

        return results;
    }

    std::vector<std::vector<BoostVertex>> dbscan(const Graph& g, float radius, std::size_t min_pts) {
        using Cluster = std::vector<BoostVertex>;
        const auto n_vertices = boost::num_vertices(g.g);

        std::vector<Cluster> clustering;
        std::size_t current_cluster = 0u;

        const long int LabelUndefined = -2;
        const long int LabelNoise = -1;
        std::map<BoostVertex, long int> label;

        // We keep the depot (vertex 0) out from the clusters.
        label[0u] = LabelNoise;

        for(auto i = 1u; i < n_vertices; ++i) {
            // We don't care about unreachable vertices, so we just
            // label them as noise from the beginning.
            label[i] = (g.g[i].reachable ? LabelUndefined : LabelNoise);
        }

        for(auto i = 1u; i < n_vertices; ++i) {
            // Vertex already assigned (or marked as noise).
            if(label[i] != LabelUndefined) { continue; }

            const auto& v = g.g[i];
            const auto pt = BoostPoint(v.x, v.y);
            const auto pt_val = std::make_pair(pt, i);
            auto neighbours = within_radius(pt, radius, g.rtree);

            // Not enough neighbours: mark as an outlier (noise).
            if(neighbours.size() < min_pts) {
                label[i] = LabelNoise;
                continue;
            }

            label[i] = current_cluster;

            // Remove i from its neighbouts.
            neighbours.erase(
              std::remove_if(
                  neighbours.begin(),
                  neighbours.end(),
                  [&pt_val] (const BoostTreeValue& val) { return pt_val.second == val.second; }
              ),
              neighbours.end()
            );

            while(!neighbours.empty()) {
                // Take one element in neighbours.
                const auto qt_val = neighbours.back();
                const auto& qt = qt_val.first;
                const auto& j = qt_val.second;

                neighbours.pop_back();

                // Skip the depot.
                if(j == 0u) {
                    assert(g.g[j].depot);
                    continue;
                }

                // Skip unreachable vertices.
                if(!g.g[j].reachable) { continue; }

                if(label[j] == LabelNoise) { label[j] = current_cluster; }
                if(label[j] != LabelUndefined) { continue; }

                label[j] = current_cluster;

                const auto further_neighbours = within_radius(qt, radius, g.rtree);

                if(further_neighbours.size() >= min_pts) {
                    for(const auto& n : further_neighbours) {
                        neighbours.push_back(n);
                    }
                }
            }

            ++current_cluster;
        }

        const auto n_clusters = current_cluster;

        if(n_clusters == 0u) {
            std::cout << as::console::warning << "DBSCAN could not create any cluster." << std::endl;
            return clustering;
        }

        clustering.resize(n_clusters);

        for(auto k = 0u; k < n_clusters; ++k) {
            clustering[k] = Cluster();
        }

        for(auto i = 0u; i < n_vertices; ++i) {
            const auto k = label[i];
            assert(k != LabelUndefined);

            if(k != LabelNoise) {
                clustering[k].push_back(i);
            }
        }

        assert(std::all_of(
           clustering.begin(),
           clustering.end(),
           [min_pts] (const std::vector<BoostVertex>& cluster) -> bool {
               return cluster.size() >= min_pts;
           }
        ));

        std::cout << as::console::notice << "DBSCAN created " << n_clusters << " clusters." << std::endl;

        return clustering;
    }

    std::vector<std::vector<BoostVertex>> dbscan(const Graph& g) {
        // Get the distance of each vertex to its nearest neighbour.
        const auto distances = nearest_neighbour_distances(g);

        // Get the smallest distance.
        const auto radius = distances.back();

        std::cout << as::console::notice << "DBSCAN auto-tuned radius: " << radius << std::endl;

        // Now we know that 95% of the vertices have their nearest
        // neighbour within this radius. We now check how many points
        // lie within each vertex's neighbourhood, using this radius.
        // In other words, we want to know how many points will be
        // in each neighbourhood.
        const auto nb_sizes = neighbourhood_sizes(radius, g);

        // We divide these sizes in 20 equally spaces buckets and
        // count how many of them falls in each bucket.
        std::vector<std::vector<std::size_t>> buckets(20u, std::vector<std::size_t>());
        std::size_t interval = nb_sizes.back() - nb_sizes.front();

        for(const auto& sz : nb_sizes) {
            const float placement = static_cast<float>(sz - nb_sizes.front()) / static_cast<float>(interval);
            buckets[placement * 19u].push_back(sz);
        }

        // We expect to have a lot of vertices in the first bucket,
        // corresponding with 0 or very few vertices in their
        // neighbourhood. Once these are passed, then the buckets
        // should follow some kind of triangular distribution.
        // E.g., if buckets had size 7, we would expect:
        //
        // buckets[6] = ##
        // buckets[5] = ####
        // buckets[4] = #######
        // buckets[3] = #############
        // buckets[2] = ##########
        // buckets[1] = ######
        // buckets[0] = #################
        //
        // So, scanning buckets from 0 to 19, we expect to first
        // see the size of the buckets shrink quickly, then grow,
        // then shrink again. We want to take the last bucket in
        // the "shrink quickly" sequence. E.g., in the example
        // above, this would be buckets[1].

        std::size_t bucket_id = 0u;
        while(bucket_id < buckets.size() - 1u) {
            if(
                buckets[bucket_id].empty() ||
                (
                    !buckets[bucket_id + 1].empty() &&
                    buckets[bucket_id + 1].size() < buckets[bucket_id].size()
                )
            ) {
                ++bucket_id;
            } else {
                break;
            }
        }

        // We take as number of points, the highest number in
        // the chosen bucket.
        auto min_pts = buckets[bucket_id].back();

        // Anyways, we need at least 2 points to create a cluster.
        min_pts = std::max(min_pts, 2ul);

        std::cout << as::console::notice << "DBSCAN auto-tuned min_pts: " << min_pts << std::endl;

        // Finally, we run the clustering algorithm with these parameters.
        return dbscan(g, radius, min_pts);
    }

    std::vector<float> nearest_neighbour_distances(const Graph& g) {
        std::vector<float> dist;

        for(const auto& vertex : as::graph::vertices(g.g)) {
            const auto out_edges = as::graph::out_edges(vertex, g.g);

            if(out_edges.begin() == out_edges.end()) {
                // No neighbours.
                continue;
            }

            const auto shortest_edge = *std::min_element(
                  out_edges.begin(),
                  out_edges.end(),
                  [&g] (const BoostEdge& e1, const BoostEdge& e2) -> bool {
                      return g.g[e1].travel_time < g.g[e2].travel_time;
                  });

            dist.push_back(g.g[shortest_edge].travel_time);
        }

        std::sort(dist.begin(), dist.end());

        return dist;
    }

    std::vector<std::size_t> neighbourhood_sizes(float radius, const Graph& g) {
        std::vector<std::size_t> sizes;

        for(const auto& vertex : as::graph::vertices(g.g)) {
            const auto& vprop = g.g[vertex];
            const auto nb = within_radius(BoostPoint(vprop.x, vprop.y), radius, g.rtree);

            std::size_t nb_size = 0u;

            // Calculate neighbourhood size excluding the depot and
            // unreachable vertices.
            for(const auto& neighbour : nb) {
                const auto& n_vertex = boost::vertex(neighbour.second, g.g);
                const auto& n_vprop = g.g[n_vertex];

                if(!n_vprop.depot && n_vprop.reachable) {
                    ++nb_size;
                }
            }

            sizes.push_back(nb_size);
        }

        std::sort(sizes.begin(), sizes.end());

        return sizes;
    }
}
