#ifndef LAYERED_GRAPH_STRATEGY_HPP
#define LAYERED_GRAPH_STRATEGY_HPP

#include "gpu_voxel_planning/strategies/victor_selective_densification.hpp"
#include "gpu_voxel_planning/strategies/strategies.hpp"
#include "gpu_voxel_planning/strategies/memorized_swept_volumes.hpp"
#include <arc_utilities/timing.hpp>
#include "gpu_voxel_planning/sd_params.hpp"


namespace GVP
{
    typedef int64_t NodeIndex;

    class LayeredGraphStrategy : public Strategy
    {
    public:
        bool initialized = false;
        MemorizedSweptVolume precomputed_swept_volumes;
        const std::string graph_filepath;
        const std::string swept_volumes_filepath;

        double discretization = SD_EDGE_DISCRETIZATION;

        int vized_id = 0;
        GpuVoxelRvizVisualizer* viz;

        enum EdgeCheckMode {FAST, STORE};
        EdgeCheckMode mode;

        NodeIndex cur_node;
        NodeIndex goal_node;
        
        Path start_to_graph, graph_to_goal;

    public:
        LayeredGraphStrategy(const std::string &graph_filepath,
                             const std::string& swept_volumes_filepath);
        
        void setMode(EdgeCheckMode mode_);

        bool checkPathFast(VictorRightArmConfig q_start, VictorRightArmConfig q_end, State &s);

        void saveToFile(std::string filename);
        void saveToFile() {saveToFile(swept_volumes_filepath);}

        virtual arc_dijkstras::GraphEdge& getReverseEdge(arc_dijkstras::GraphEdge &e) = 0;
        virtual VictorRightArmConfig getStart(arc_dijkstras::GraphEdge &e) = 0;
        virtual VictorRightArmConfig getEnd(arc_dijkstras::GraphEdge &e) = 0;
        virtual std::vector<double> getNodeValue(NodeIndex ind) = 0;
        virtual int getDepth(arc_dijkstras::GraphEdge &e) = 0;

        virtual std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s) = 0;

        DenseGrid computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e);
        void storeSweptVolume(const arc_dijkstras::GraphEdge &e, const DenseGrid &g);
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

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) = 0;

        /* Strategy Overrides */
        virtual Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz) override;

        /* Strategy Helpers */
        virtual void initialize(Scenario &scenario) = 0;

        double evaluateEdge(arc_dijkstras::GraphEdge &e, State &s);
    };


    
}





#endif //LAYERED_GRAPH_STRATEGY_HPP
