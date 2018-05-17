//
// Created by alberto on 19/10/17.
//

#include <as/graph.h>
#include <as/console.h>
#include <as/containers.h>
#include "Clustering.h"
#include "Plotter.h"
#include "RTreeUtils.h"

namespace op {
    namespace fs = std::experimental::filesystem;
    using namespace cimg_library;
    using namespace as;

    Plotter::Plotter(const Graph& graph) : graph{graph} {
        initialise_canvas();
        add_clustered_graph_to_canvas();
        add_vertex_ids_to_canvas();
    }

    void Plotter::initialise_canvas() {
        // Image scaling compared to the (x,y)-dimensions reported by the instance file.
        scaling = 1.0f;

        const auto x_dim = static_cast<unsigned int>(graph.max_x - graph.min_x);
        const auto y_dim = static_cast<unsigned int>(graph.max_y - graph.min_y);

        // If none of the dimensions is at least 1000 pixels, we scale the image
        // so that the x dimension will be 1000 pixels (+ 2 * padding).
        if(x_dim < min_size && y_dim < min_size) {
            std::cout << console::warning << "Image dimensions: " << x_dim << " x " << y_dim << "; scaling up." << std::endl;
            scaling = static_cast<float>(min_size) / x_dim;
        }

        // If the image is too large, CImg will crash when saving to png!
        if(x_dim > max_size || y_dim > max_size) {
            std::cout << console::warning << "Image dimensions: " << x_dim << " x " << y_dim << "; scaling down." << std::endl;
            scaling = static_cast<float>(max_size) / std::max(x_dim, y_dim);
        }

        img = Image(
            static_cast<unsigned int>(x_dim * scaling) + 2u * padding, // x-dimension
            static_cast<unsigned int>(y_dim * scaling) + 2u * padding, // y-dimension
            1u,                                                        // z-dimension
            3u                                                         // colour depth (3 for RGB)
        );

        const unsigned char white[] = { 255, 255, 255 };

        // Draw a white background.
        cimg_forXYZ(img, x, y, z) { img.fillC(x, y, z, white[0], white[1], white[2]); }
    }

    void Plotter::add_clustered_graph_to_canvas() {
        using as::containers::contains;

        const std::array<unsigned char, 3> gray = { 150, 150, 150 };
        const std::array<unsigned char, 3> black = { 0, 0, 0 };
        const std::vector<std::array<unsigned char, 3>> colours = {
            { 60, 180, 75 },    // Green
            { 255, 255, 25 },   // Yellow
            { 0, 130, 200 },    // Blue
            { 245, 130, 48 },   // Orange
            { 145, 30, 180 },   // Purple
            { 70, 240, 240 },   // Cyan
            { 240, 50, 230 },   // Magenta
            { 210, 245, 60 },   // Lime
            { 250, 190, 190 },  // Pink
            { 0, 128, 128 },    // Teal
            { 230, 190, 255}    // Lavender
        };

        Clustering c(&graph);
        std::vector<BoostVertex> clustered;

        for(auto k = 0u; k < c.n_clusters; ++k) {
            const auto colour = colours[k % colours.size()];

            for(const auto& vertex : c.clusters[k]) {
                clustered.push_back(vertex);

                const auto x = get_x(vertex), y = get_y(vertex), radius = get_radius(vertex);
                img.draw_circle(x, y, radius, graph.g[vertex].reachable ? colour.data() : gray.data());
            }
        }

        // Colour all other vertices black.
        for(const auto& vertex : as::graph::vertices(graph.g)) {
            if(!contains(clustered, vertex)) {
                const auto x = get_x(vertex), y = get_y(vertex), radius = get_radius(vertex);
                img.draw_circle(x, y, radius, graph.g[vertex].reachable ? black.data() : gray.data());
            }
        }
    }

    void Plotter::add_vertex_ids_to_canvas() {
        for(const auto& vertex : as::graph::vertices(graph.g)) {
            const auto x = get_x(vertex), y = get_y(vertex);
            const unsigned char red[] = { 255, 0, 0 };
            img.draw_text(x + 5, y + 5, "%zu", red, NULL, 1, 13, vertex);
        }
    }

    void Plotter::plot_graph_to_png(const fs::path& image_file) const {
        img.save_png(image_file.c_str());
        std::cout << as::console::notice << "Saved graph image to file: " << image_file << std::endl;
    }

    void Plotter::plot_donut_to_png(BoostVertex v, float min_r, float max_r, const std::experimental::filesystem::path& image_file) const {
        auto cimg = img;

        const unsigned char red[] = {255, 0, 0};

        cimg.draw_circle(get_x(v), get_y(v), min_r * scaling, red, 1.0, 1u);
        cimg.draw_circle(get_x(v), get_y(v), max_r * scaling, red, 1.0, 1u);

        cimg.save_png(image_file.c_str());
        std::cout << as::console::notice << "Saved donut image to file: " << image_file << std::endl;
    }

    void Plotter::plot_tour_to_png(const Tour& tour, const fs::path& image_file) const {
        // Make a local copy of the image object, so that it can be reused.
        auto cimg = img;

        const unsigned char red[] = { 255, 0, 0 };
        // const unsigned char black[] = { 0, 0, 0 };

        for(const auto& edge : tour.edges) {
            const auto v_orig = boost::source(edge, graph.g);
            const auto v_dest = boost::target(edge, graph.g);
            const auto x_orig = get_x(v_orig), y_orig = get_y(v_orig);
            const auto x_dest = get_x(v_dest), y_dest = get_y(v_dest);

            // Draw a thick line:
            cimg.draw_line(x_orig, y_orig, x_dest, y_dest, red);
            cimg.draw_line(x_orig-1, y_orig, x_dest-1, y_dest, red);
            cimg.draw_line(x_orig, y_orig-1, x_dest, y_dest-1, red);

            // Draw edge travel time:
            // cimg.draw_text(
            //     (x_orig + x_dest) / 2 - 5,
            //     (y_orig + y_dest) / 2 - 5,
            //     "%.2f",
            //     black,
            //     NULL,
            //     1,
            //     13,
            //     graph.g[edge].travel_time
            // );
        }

        cimg.save_png(image_file.c_str());

        std::cout << as::console::notice << "Saved tour image to file: " << image_file << std::endl;
    }

    unsigned int Plotter::get_x(const BoostVertex& v) const {
        return static_cast<unsigned int>((graph.g[v].x - graph.min_x) * scaling) + padding;
    };

    unsigned int Plotter::get_y(const BoostVertex& v) const {
        return static_cast<unsigned int>((graph.g[v].y - graph.min_y) * scaling) + padding;
    };

    unsigned int Plotter::get_radius(const BoostVertex& v) const {
        if(graph.min_prize == graph.max_prize) {
            // All prizes are the same.
            return static_cast<unsigned int>(min_radius);
        }

        if(graph.g[v].depot) {
            return static_cast<unsigned int>(min_radius);
        }

        auto prize = graph.g[v].prize;
        auto radius = min_radius + (max_radius - min_radius) * ((prize - graph.min_prize) / (graph.max_prize - graph.min_prize));
        return static_cast<unsigned int>(radius);
    };
}
