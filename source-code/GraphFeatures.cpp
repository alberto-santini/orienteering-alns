//
// Created by alberto on 12/02/18.
//

#include <as/graph.h>
#include <cmath>

#include "GraphFeatures.h"

namespace op {
    namespace features {
        float diameter(const Graph& graph) {
            float max_diam = 0;

            for(const auto& e : as::graph::edges(graph.g)) {
                if(graph.g[e].travel_time > max_diam) {
                    max_diam = graph.g[e].travel_time;
                }
            }

            return max_diam / graph.max_travel_time;
        }

        float max_distance_from_depot(const Graph& graph) {
            const auto depot = 0u;
            float max_dist = 0;

            for(const auto& e : as::graph::out_edges(depot, graph.g)) {
                if(graph.g[e].travel_time > max_dist) {
                    max_dist = graph.g[e].travel_time;
                }
            }

            return max_dist / graph.max_travel_time;
        }

        float distance_btw_barycentre_and_depot(const Graph& graph) {
            using namespace as::geo;

            const float nv = -1.0f + boost::num_edges(graph.g);
            float tot_x = 0, tot_y = 0;

            for(const auto& v : as::graph::vertices(graph.g)) {
                if(!graph.g[v].depot) {
                    tot_x += graph.g[v].x * graph.g[v].prize;
                    tot_y += graph.g[v].y * graph.g[v].prize;
                }
            }

            TwoDimPoint barycentre{tot_x / nv, tot_y / nv};
            TwoDimPoint depot{graph.g[0u].x, graph.g[0u].y};

            return euclidean_distance(barycentre, depot) / graph.max_travel_time;
        }

        float number_of_clusters_frac(const Clustering& clustering) {
            assert(clustering.graph != nullptr);

            const float clusterable_vertices = -1.0f + clustering.graph->n_vertices;
            return static_cast<float>(clustering.n_clusters) / clusterable_vertices;
        }

        namespace {
            float avg_cluster_size(const Clustering& clustering) {
                std::size_t tot_size = 0u;

                for(const auto& cluster : clustering.clusters) {
                    tot_size += cluster.size();
                }

                return static_cast<float>(tot_size) / static_cast<float>(clustering.n_clusters);
            }
        }

        float avg_cluster_size_frac(const Clustering& clustering) {
            assert(clustering.graph != nullptr);

            const float clusterable_vertices = -1.0f + clustering.graph->n_vertices;
            return avg_cluster_size(clustering) / clusterable_vertices;
        }

        namespace {
            float cluster_diameter(const Clustering& clustering, std::size_t cluster_id) {
                assert(cluster_id < clustering.n_clusters);
                assert(clustering.graph != nullptr);

                const auto& cluster = clustering.clusters[cluster_id];
                float max_diam = 0;

                for(auto i = 0u; i < cluster.size(); ++i) {
                    for(auto j = i + 1; j < cluster.size(); ++j) {
                        const float dist = clustering.graph->travel_time(cluster[i], cluster[j]);

                        if(dist > max_diam) {
                            max_diam = dist;
                        }
                    }
                }

                return max_diam;
            }

            float avg_cluster_diameter(const Clustering& clustering) {
                const float nc = clustering.n_clusters;
                float tot_diam = 0;

                for(auto i = 0u; i < clustering.n_clusters; ++i) {
                    tot_diam += cluster_diameter(clustering, i);
                }

                return tot_diam / nc;
            }
        }

        float avg_cluster_diameter_frac(const Clustering& clustering) {
            assert(clustering.graph != nullptr);
            return avg_cluster_diameter(clustering) / clustering.graph->max_travel_time;
        }

        namespace {
            float avg_distance(const std::vector<as::geo::TwoDimPoint>& points) {
                float tot_dist = 0.0f;

                for(auto i = 0u; i < points.size(); ++i) {
                    for(auto j = i + 1; j < points.size(); ++j) {
                        tot_dist += as::geo::euclidean_distance(points[i], points[j]);
                    }
                }

                const float n_samples = points.size() * (points.size() - 1.0f);
                return tot_dist / n_samples;
            }

            float graph_spread(const Graph& graph) {
                std::vector<as::geo::TwoDimPoint> points;
                points.reserve(graph.n_vertices);

                for(const auto& v : as::graph::vertices(graph.g)) {
                    points.push_back(as::geo::TwoDimPoint{graph.g[v].x, graph.g[v].y});
                }

                return avg_distance(points);
            }

            float cluster_spread(const Clustering& clustering) {
                std::vector<as::geo::TwoDimPoint> points;
                points.reserve(clustering.n_clusters);

                for(const auto& pt : clustering.centres) {
                    points.push_back(as::geo::TwoDimPoint{pt.x(), pt.y()});
                }

                return avg_distance(points);
            }

            float cluster_and_isolated_spread(const Clustering& clustering) {
                std::vector<as::geo::TwoDimPoint> points;
                points.reserve(clustering.n_clusters + clustering.noise.size());

                for(const auto& pt : clustering.centres) {
                    points.push_back(as::geo::TwoDimPoint{pt.x(), pt.y()});
                }

                for(const auto& v : clustering.noise) {
                    points.push_back(as::geo::TwoDimPoint{clustering.graph->g[v].x, clustering.graph->g[v].y});
                }

                return avg_distance(points);
            }
        }

        float graph_spread_frac(const Graph& graph) {
            return graph_spread(graph) / graph.max_travel_time;
        }

        float cluster_spread_frac(const Clustering& clustering) {
            assert(clustering.graph != nullptr);
            return cluster_spread(clustering) / clustering.graph->max_travel_time;
        }

        float cluster_and_isolated_spread_frac(const Clustering& clustering) {
            assert(clustering.graph != nullptr);
            return cluster_and_isolated_spread(clustering) / clustering.graph->max_travel_time;
        }

        float isolated_vertices_frac(const Clustering& clustering) {
            assert(clustering.graph != nullptr);

            const float n_isolated = clustering.noise.size();
            const float n_clusterable = clustering.graph->n_vertices - 1.0f;

            return n_isolated / n_clusterable;
        }

        namespace {
            namespace bgi = boost::geometry::index;

            BoostBox graph_bounding_rectangle(const Graph& graph) {
                float min_x = std::numeric_limits<float>::max();
                float min_y = std::numeric_limits<float>::max();
                float max_x = std::numeric_limits<float>::lowest();
                float max_y = std::numeric_limits<float>::lowest();

                for(const auto& v_id : as::graph::vertices(graph.g)) {
                    const auto& v = graph.g[v_id];
                    if(v.x < min_x) { min_x = v.x; }
                    if(v.x > max_x) { max_x = v.x; }
                    if(v.y < min_y) { min_y = v.y; }
                    if(v.y > max_y) { max_y = v.y; }
                }

                return BoostBox{BoostPoint{min_x, min_y}, BoostPoint{max_x, max_y}};
            }

            float strip_width(const Graph& graph) {
                return graph.max_travel_time / static_cast<float>(graph.n_vertices);
            }

            std::size_t pts_in_horizontal_strip(float start_y, float width, const BoostBox& br, const Graph& graph) {
                std::vector<BoostTreeValue> points;

                // Removing epsilon to avoid counting points exactly at the intersection of two strips twice.
                const BoostBox bounding(
                    BoostPoint(br.min_corner().x(), start_y),
                    BoostPoint(br.max_corner().x(), start_y + width - std::numeric_limits<float>::epsilon())
                );
                graph.rtree.query(bgi::within(bounding), std::back_inserter(points));

                return points.size();
            }

            std::size_t pts_in_vertical_strip(float start_x, float width, const BoostBox& br, const Graph& graph) {
                std::vector<BoostTreeValue> points;

                // Removing epsilon to avoid counting points exactly at the intersection of two strips twice.
                const BoostBox bounding(
                    BoostPoint(start_x, br.min_corner().y()),
                    BoostPoint(start_x + width - std::numeric_limits<float>::epsilon(), br.max_corner().y())
                );
                graph.rtree.query(bgi::within(bounding), std::back_inserter(points));

                return points.size();
            }

            std::size_t pts_in_diagonal_strip(float start_intercept, float width, const Graph& graph) {
                std::size_t n_points = 0u;

                for(const auto& v_id : as::graph::vertices(graph.g)) {
                    const auto& v = graph.g[v_id];

                    if(v.x + start_intercept <= v.y &&
                       v.y <= v.x + start_intercept + width
                        ) {
                        ++n_points;
                    }
                }
                return n_points;
            }

            std::size_t pts_in_reverse_diagonal_strip(float start_intercept, float width, const Graph& graph) {
                std::size_t n_points = 0u;

                for(const auto& v_id : as::graph::vertices(graph.g)) {
                    const auto& v = graph.g[v_id];

                    if(-v.x + start_intercept <= v.y &&
                       v.y <= -v.x + start_intercept + width
                        ) {
                        ++n_points;
                    }
                }
                return n_points;
            }

            std::size_t largest_horizontal_intersection(const Graph& graph) {
                const auto br = graph_bounding_rectangle(graph);
                const float width = strip_width(graph);
                float cur_y = br.min_corner().y();
                std::size_t largest = 0u;

                do {
                    auto n = pts_in_horizontal_strip(cur_y, width, br, graph);
                    if(n > largest) {
                        largest = n;
                    }
                    cur_y += width;
                } while(cur_y < br.max_corner().y());

                return largest;
            }

            std::size_t largest_vertical_intersection(const Graph& graph) {
                const auto br = graph_bounding_rectangle(graph);
                const float width = strip_width(graph);
                float cur_x = br.min_corner().x();
                std::size_t largest = 0u;

                do {
                    auto n = pts_in_vertical_strip(cur_x, width, br, graph);
                    if(n > largest) {
                        largest = n;
                    }
                    cur_x += width;
                } while(cur_x < br.max_corner().x());

                return largest;
            }

            std::size_t largest_diagonal_intersection(const Graph& graph) {
                const auto br = graph_bounding_rectangle(graph);
                const float width = strip_width(graph);
                float cur_intercept = br.min_corner().y() - br.max_corner().x();
                const float max_intercept = br.max_corner().y() - br.min_corner().x();
                const float intercept_increment = static_cast<float>(M_SQRT2) * width;
                std::size_t largest = 0u;

                assert(max_intercept >= cur_intercept);

                do {
                    auto n = pts_in_diagonal_strip(cur_intercept, width, graph);
                    if(n > largest) {
                        largest = n;
                    }
                    cur_intercept += intercept_increment;
                } while(cur_intercept < max_intercept);

                return largest;
            }

            std::size_t largest_reverse_diagonal_intersection(const Graph& graph) {
                const auto br = graph_bounding_rectangle(graph);
                const float width = strip_width(graph);
                float cur_intercept = br.min_corner().y() + br.min_corner().x();
                const float max_intercept = br.max_corner().y() + br.max_corner().x();
                const float intercept_increment = static_cast<float>(M_SQRT2) * width;
                std::size_t largest = 0u;

                assert(max_intercept >= cur_intercept);

                do {
                    auto n = pts_in_reverse_diagonal_strip(cur_intercept, width, graph);
                    if(n > largest) {
                        largest = n;
                    }
                    cur_intercept += intercept_increment;
                } while(cur_intercept <= max_intercept);

                return largest;
            }

            std::size_t largest_set_of_aligned_pts(const Graph& graph) {
                const std::vector<std::size_t> sizes({
                                                         largest_horizontal_intersection(graph),
                                                         largest_vertical_intersection(graph),
                                                         largest_diagonal_intersection(graph),
                                                         largest_reverse_diagonal_intersection(graph)
                });

                return *std::max_element(sizes.begin(), sizes.end());
            }
        }

        float largest_set_of_aligned_pts_frac(const Graph& graph) {
            return static_cast<float>(largest_set_of_aligned_pts(graph)) / static_cast<float>(graph.n_vertices);
        }

        void print_features(std::ostream& out, const Graph& graph, std::experimental::observer_ptr<Clustering> clustering) {
            out << diameter(graph) << ",";
            out << max_distance_from_depot(graph) << ",";
            out << distance_btw_barycentre_and_depot(graph) << ",";
            out << graph_spread_frac(graph) << ",";
            out << largest_set_of_aligned_pts_frac(graph) << ",";

            if(clustering != nullptr) {
                out << number_of_clusters_frac(*clustering) << ",";
                out << avg_cluster_size_frac(*clustering) << ",";
                out << avg_cluster_diameter_frac(*clustering) << ",";
                out << cluster_spread_frac(*clustering) << ",";
                out << cluster_and_isolated_spread_frac(*clustering) << ",";
                out << isolated_vertices_frac(*clustering);
            } else {
                out << "0,0,0,0,0,0";
            }

            out << "\n";
        }
    }
}