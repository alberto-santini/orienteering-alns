//
// Created by alberto on 19/10/17.
//

#ifndef OP_PLOTTER_H
#define OP_PLOTTER_H

#include "Graph.h"
#include "Tour.h"
#include <experimental/filesystem>
#include <CImg.h>

namespace op {
    /**
     * The plotter produces nice visualisation of instances
     * and solutions.
     */
    class Plotter {
        using Image = cimg_library::CImg<unsigned char>;

        /**
         * The underlying graph.
         */
        const Graph& graph;

        /**
         * The image object.
         */
        Image img;

        /**
         * Scaling factor applied
         */
        float scaling;

        /**
         * Number of pixels to leave as a border.
         */
        static constexpr unsigned int padding = 20u;

        /**
         * Images whose dimensions are both smaller than this are scaled up.
         */
        static constexpr unsigned int min_size = 1000u;

        /**
         * Images with at least one dimension larger than this are scaled down.
         */
        static constexpr unsigned int max_size = 5000u;

        /**
         * The vertex with the smallest prize will be drawn with a ball of this radius.
         */
        static constexpr float min_radius = 3.0f;

        /**
         * The vertex with the highest prize will be drawn with a ball of this radius.
         */
        static constexpr float max_radius = 15.0f;

    public:

        /**
         * Creates an image for the specified graph.
         *
         * @param graph The underlying graph.
         */
        explicit Plotter(const Graph& graph);

        /**
         * Saves the clustered graph's picture to a .png file.
         *
         * @param image_file Image where to save the plot.
         */
        void plot_graph_to_png(const std::experimental::filesystem::path& image_file) const;

        /**
         * Plots a tour on top of the clustered graph, and saves
         * the resulting image to a .png file.
         *
         * @param tour       The tour to plot.
         * @param image_file Image where we save the plot.
         */
        void plot_tour_to_png(const Tour& tour, const std::experimental::filesystem::path& image_file) const;

        /**
         * Plots a donut around a vertex, and saves the resulting
         * image to a .png file.
         *
         * @param v             The vertex.
         * @param min_r         The minimum radius.
         * @param max_r         The maximum radius.
         * @param image_file    The image where to save the plot.
         */
        void plot_donut_to_png(BoostVertex v, float min_r, float max_r, const std::experimental::filesystem::path& image_file) const;

    private:
        /**
         * Initialises a white canvas of the right dimensions to accommodate
         * the graph.
         */
        void initialise_canvas();

        /**
         * Prints the clustered graph to the image.
         */
        void add_clustered_graph_to_canvas();

        /**
         * Prints the vertex ids to the image.
         */
        void add_vertex_ids_to_canvas();

        /**
         * Get the x-coordinate of the centre pixel for a vertex.
         * @param v     The vertex.
         * @return      The coordinate.
         */
        unsigned int get_x(const BoostVertex& v) const;

        /**
         * Get the y-coordinate of the centre pixel for a vertex.
         * @param v     The vertex.
         * @return      The coordinate.
         */
        unsigned int get_y(const BoostVertex& v) const;

        /**
         * Get the radius of the circle representing a vertex.
         * @param v     The vertex.
         * @return      The radius.
         */
        unsigned int get_radius(const BoostVertex& v) const;
    };
}

#endif //OP_PLOTTER_H
