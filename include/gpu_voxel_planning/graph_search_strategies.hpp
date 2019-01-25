#ifndef GRAPH_SEARCH_STRATEGIES_HPP
#define GRAPH_SEARCH_STRATEGIES_HPP

#include "victor_halton_roadmap.hpp"
#include "state.hpp"
#include "strategies.hpp"
#include <graph_planner/dijkstras_addons.hpp>
#include <cmath>
#include <arc_utilities/timing.hpp>
#include "memorized_swept_volumes.hpp"

namespace GVP
{

    typedef int64_t NodeIndex;
    
    class GraphSearchStrategy : public Strategy
    {
    public:        
        Roadmap graph;
        NodeIndex prev_node;
        NodeIndex cur_node;
        NodeIndex goal_node;
        bool initialized;
        double discretization = 0.02;
        const std::string graph_filepath;
        const std::string swept_volumes_filepath;
        

        // std::map<arc_dijkstras::HashableEdge, SparseGrid> precomputed_swept_volumes;
        MemorizedSweptVolume precomputed_swept_volumes;

        // GraphSearchStrategy(const std::string &filename) : graph(filename), initialized(false) {}
        GraphSearchStrategy(const std::string &graph_filepath, const std::string& swept_volumes_filepath);
        GraphSearchStrategy(const std::string &graph_filepath);
        GraphSearchStrategy();

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) = 0;

        void initialize(const Scenario &scenario);

        virtual Path applyTo(Scenario &scenario) override final;

        void computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e);

        DenseGrid getSweptVolume(State &s, arc_dijkstras::GraphEdge &e);

        
        /* Checks an edge against known obstacles to see if there is a collision.
         * Changes the graph edge validity accordingly
         */
        bool checkEdge(arc_dijkstras::GraphEdge &e, State &s);

        double evaluateEdge(arc_dijkstras::GraphEdge &e, State &s);

        std::vector<NodeIndex> lazySp(NodeIndex start, NodeIndex goal, State &s);

        void saveToFile(std::string filename);
        void saveToFile() {saveToFile(swept_volumes_filepath);}

    };


    
    class OptimisticGraphSearch : public GraphSearchStrategy
    {
    public:        

        OptimisticGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}
        OptimisticGraphSearch() {}

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override;

        virtual std::string getName() const override;
    };        



    class ParetoCostGraphSearch : public GraphSearchStrategy
    {
    public:        
        double alpha = 10.0;
        ParetoCostGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}
        ParetoCostGraphSearch(double alpha) : alpha(alpha){}
        
        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override;

        virtual std::string getName() const override;
    };

    class UnknownSpaceCostGraphSearch : public GraphSearchStrategy
    {
    public:
        double alpha;
        double free_cost;
        UnknownSpaceCostGraphSearch(double alpha, double free_cost) :
            alpha(alpha), free_cost(free_cost){}

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override;

        virtual std::string getName() const override;
    };
}

#endif



