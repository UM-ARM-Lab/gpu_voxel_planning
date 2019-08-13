#ifndef SELECTIVE_DENSIFICATION_STRATEGIES_HPP
#define SELECTIVE_DENSIFICATION_STRATEGIES_HPP

#include "strategies/victor_selective_densification.hpp"
#include "layered_graph_strategy.hpp"
#include "strategies/strategies.hpp"
#include "strategies/memorized_swept_volumes.hpp"
#include <arc_utilities/timing.hpp>
#include "sd_params.hpp"



/* 
 * 
 */
namespace GVP
{
    //Currently only suitible for known omnicient environments
    //Duplicates some functions from "graph_search_strategies", but works with SDRoadmap instead of Roadmap
    class SelectiveDensificationStrategy : public Strategy
    {

    public:
        SDRoadmap sd_graph;
        MemorizedSweptVolume precomputed_swept_volumes;
        bool initialized;
        double discretization = SD_EDGE_DISCRETIZATION;
        const std::string graph_filepath;
        const std::string swept_volumes_filepath;
        NodeIndex cur_node;
        NodeIndex goal_node;
        GpuVoxelRvizVisualizer* viz;
        int vized_id = 0;

        Path start_to_graph, graph_to_goal;
        enum EdgeCheckMode {FAST, STORE};

        EdgeCheckMode mode;

    public:


        SelectiveDensificationStrategy(const std::string &graph_filepath,
                                       const std::string& swept_volumes_filepath);
        // SelectiveDensificationStrategy(const std::string &graph_filepath);
        SelectiveDensificationStrategy();

        void initialize(Scenario &scenario);

        /*
         *  Adds nodes and edges to the graph corresponding to the start and goal of the scenario
         */
        void addStartAndGoalToGraph(const Scenario &scenario);

        // /*
        //  *  Finds paths from the scenario start and goal to the graph (but does not add new nodes or edges)
        //  */
        // void connectStartAndGoalToGraph(Scenario &scenario);

        // NodeIndex connectToGraph(Scenario &scenario, const VictorRightArmConfig &q);

        void setMode(EdgeCheckMode mode_);

        virtual Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz) override;

        virtual std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s);

        virtual DenseGrid getSweptVolume(State &s, arc_dijkstras::GraphEdge &e);

        void vizEdge(arc_dijkstras::GraphEdge &e);

        /* Checks the swept volume an edge against known obstacles to see if there is a collision.
         *  Depending on the mode, this may check the full edge and memorize the swept volume, 
         *  or may terminate early if a collision is found
         */
        bool checkEdge(arc_dijkstras::GraphEdge &e, State &s);
        
        /* Checks the swept volume an edge against known obstacles to see if there is a collision.
         * Changes the graph edge validity accordingly
         * Note: This gets the entire swept volume of the edge first first
         */
        bool checkEdgeAndStore(arc_dijkstras::GraphEdge &e, State &s);

        /* Checks the swept volume an edge one configuration at at time against 
         *  known obstacles to see if there is a collision.
         * Changes the graph edge validity accordingly
         * Note: This returns early if there is a collision and does not store the swept volume
         */
        bool checkEdgeFast(arc_dijkstras::GraphEdge &e, State &s);

        bool checkPathFast(VictorRightArmConfig q_start, VictorRightArmConfig q_end, State &s);

        double evaluateEdge(arc_dijkstras::GraphEdge &e, State &s);

        std::vector<int> forwardPrecomputedSelector(std::vector<int64_t> path,
                                                    arc_dijkstras::Graph<std::vector<double>>& g,
                                                    const arc_dijkstras::EvaluatedEdges &evaluatedEdges);

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) = 0;

        virtual double distanceHeuristic(const std::vector<double> &raw1,
                                         const std::vector<double> &raw2) const = 0;

        void saveToFile(std::string filename);
        void saveToFile() {saveToFile(swept_volumes_filepath);}

        DenseGrid computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e);
        
        void storeSweptVolume(const arc_dijkstras::GraphEdge &e, const DenseGrid &g);
        
        std::vector<NodeIndex> lazySp(NodeIndex start, NodeIndex goal, State &s);
        
        std::vector<NodeIndex> astar(NodeIndex start, NodeIndex goal, State &s);
    };

    class OmniscientSDGraphSearch : public SelectiveDensificationStrategy
    {
    protected:
        bool use_precomputed;
        double c_p;
        
    public:
        OmniscientSDGraphSearch(bool use_precomputed, double c_p, int graph_num);
        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override;
        virtual std::string getName() const override;
        // virtual Path applyTo(Scenario &scenario) override;
        virtual double distanceHeuristic(const std::vector<double> &raw1,
                                         const std::vector<double> &raw2) const override;

    };

    
    class DenseGraphSearch : public OmniscientSDGraphSearch
    {
    private:
        
    public:
        DenseGraphSearch(bool use_precomputed, int graph_num);
        virtual std::string getName() const override;
        // virtual Path applyTo(Scenario &scenario) override;
        virtual double distanceHeuristic(const std::vector<double> &raw1,
                                         const std::vector<double> &raw2) const override;

    };
}




#endif
