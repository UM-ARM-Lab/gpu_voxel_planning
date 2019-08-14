#ifndef ITERATIVE_DEEPENING_STRATEGIES_HPP
#define ITERATIVE_DEEPENING_STRATEGIES_HPP

#include "layered_graph_strategy.hpp"


namespace GVP
{
    //Currently only suitible for known omnicient environments
    //Duplicates some functions from "graph_search_strategies", but works with SDRoadmap instead of Roadmap
    class IterativeDeepeningStrategy : public LayeredGraphStrategy
    {

    public:
        IDRoadmap id_graph;

    public:


        IterativeDeepeningStrategy(const std::string &graph_filepath,
                                   const std::string& swept_volumes_filepath);
        
        void initialize(Scenario &scenario) override;

        virtual arc_dijkstras::GraphEdge& getReverseEdge(arc_dijkstras::GraphEdge &e) override
        {
            return id_graph.getReverseEdge(e);
        }
        
        virtual VictorRightArmConfig getStart(arc_dijkstras::GraphEdge &e) override
        {
            return id_graph.getNodeValue(e.getFromIndex()).q;
        }
        
        virtual VictorRightArmConfig getEnd(arc_dijkstras::GraphEdge &e) override
        {
            return id_graph.getNodeValue(e.getToIndex()).q;
        }

        virtual int getDepth(arc_dijkstras::GraphEdge &e) override
        {
            return id_graph.getNodeValue(e.getFromIndex()).depth;
        }

        virtual std::vector<double> getNodeValue(NodeIndex ind) override
        {
            return id_graph.getNodeValue(ind).q;
        }


        /*
         *  Adds nodes and edges to the graph corresponding to the start and goal of the scenario
         */
        void addStartAndGoalToGraph(const Scenario &scenario);


        virtual std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s) override;

        virtual double distanceHeuristic(const std::vector<double> &raw1,
                                         const std::vector<double> &raw2) const;

        std::vector<NodeIndex> lazySp(NodeIndex start, NodeIndex goal, State &s);
    };

    class IDSearch : public IterativeDeepeningStrategy
    {
    public:
        bool use_precomputed;
        IDSearch(bool use_precomputed, int graph_num);

        std::string getName() const override;

        double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override;
    };
}

#endif //ITERATIVE_DEEPENING_STRATEGIES_HPP
