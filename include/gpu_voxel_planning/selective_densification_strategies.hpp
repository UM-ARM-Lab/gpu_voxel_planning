#ifndef SELECTIVE_DENSIFICATION_STRATEGIES_HPP
#define SELECTIVE_DENSIFICATION_STRATEGIES_HPP

#include "victor_selective_densification.hpp"
#include "strategies.hpp"
#include "memorized_swept_volumes.hpp"
#include <arc_utilities/timing.hpp>



/* 
 * 
 */
namespace GVP
{
    typedef int64_t NodeIndex;
    
    //Currently only suitible for known omnicient environments
    //Duplicates some functions from "graph_search_strategies", but works with SDRoadmap instead of Roadmap
    class SelectiveDensificationStrategy : public Strategy
    {

    public:
        SDRoadmap sd_graph;
        bool initialized;
        double discretization = 0.02;
        const std::string graph_filepath;
        const std::string swept_volumes_filepath;
        NodeIndex cur_node;
        NodeIndex goal_node;
        MemorizedSweptVolume precomputed_swept_volumes;

    public:


        SelectiveDensificationStrategy(const std::string &graph_filepath,
                                       const std::string& swept_volumes_filepath);
        SelectiveDensificationStrategy(const std::string &graph_filepath);
        SelectiveDensificationStrategy();

        void initialize(const Scenario &scenario);

        virtual Path applyTo(Scenario &scenario) override;

        virtual std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s);

        virtual DenseGrid getSweptVolume(State &s, arc_dijkstras::GraphEdge &e);

        
        /* Checks an edge against known obstacles to see if there is a collision.
         * Changes the graph edge validity accordingly
         */
        bool checkEdge(arc_dijkstras::GraphEdge &e, State &s);

        double evaluateEdge(arc_dijkstras::GraphEdge &e, State &s);

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) = 0;

        void saveToFile(std::string filename);
        void saveToFile() {saveToFile(swept_volumes_filepath);}

        DenseGrid computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e);
        
        void storeSweptVolume(const arc_dijkstras::GraphEdge &e, const DenseGrid &g);
        
        std::vector<NodeIndex> lazySp(NodeIndex start, NodeIndex goal, State &s);
    };

    class OmniscientSDGraphSearch : public SelectiveDensificationStrategy
    {
    public:
        OmniscientSDGraphSearch(const std::string &filename) : SelectiveDensificationStrategy(filename) {}
        OmniscientSDGraphSearch() {}
        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override;
        virtual std::string getName() const override;
        // virtual Path applyTo(Scenario &scenario) override;

    };
}




#endif
