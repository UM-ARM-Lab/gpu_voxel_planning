#ifndef SELECTIVE_DENSIFICATION_STRATEGIES_HPP
#define SELECTIVE_DENSIFICATION_STRATEGIES_HPP

#include "layered_graph_strategy.hpp"

/* 
 * 
 */
namespace GVP
{
    //Currently only suitible for known omnicient environments
    //Duplicates some functions from "graph_search_strategies", but works with SDRoadmap instead of Roadmap
    class SelectiveDensificationStrategy : public LayeredGraphStrategy
    {

    public:
        SDRoadmap sd_graph;

    public:


        SelectiveDensificationStrategy(const std::string &graph_filepath,
                                       const std::string& swept_volumes_filepath);
        // SelectiveDensificationStrategy(const std::string &graph_filepath);
        // SelectiveDensificationStrategy();

        void initialize(Scenario &scenario) override;

        virtual arc_dijkstras::GraphEdge& getReverseEdge(arc_dijkstras::GraphEdge &e) override
        {
            return sd_graph.getReverseEdge(e);
        }
        
        virtual VictorRightArmConfig getStart(arc_dijkstras::GraphEdge &e) override
        {
            return sd_graph.getNodeValue(e.getFromIndex()).q;
        }
        
        virtual VictorRightArmConfig getEnd(arc_dijkstras::GraphEdge &e) override
        {
            return sd_graph.getNodeValue(e.getToIndex()).q;
        }

        virtual int getDepth(arc_dijkstras::GraphEdge &e) override
        {
            return sd_graph.getNodeValue(e.getFromIndex()).depth;
        }

        virtual std::vector<double> getNodeValue(NodeIndex ind) override
        {
            return sd_graph.getNodeValue(ind).q;
        }


        /*
         *  Adds nodes and edges to the graph corresponding to the start and goal of the scenario
         */
        void addStartAndGoalToGraph(const Scenario &scenario);

        // /*
        //  *  Finds paths from the scenario start and goal to the graph (but does not add new nodes or edges)
        //  */
        // void connectStartAndGoalToGraph(Scenario &scenario);

        // NodeIndex connectToGraph(Scenario &scenario, const VictorRightArmConfig &q);

        virtual std::vector<NodeIndex> plan(NodeIndex start, NodeIndex goal, State &s) override;

        std::vector<int> forwardPrecomputedSelector(std::vector<int64_t> path,
                                                    arc_dijkstras::Graph<std::vector<double>>& g,
                                                    const arc_dijkstras::EvaluatedEdges &evaluatedEdges);

        virtual double distanceHeuristic(const std::vector<double> &raw1,
                                         const std::vector<double> &raw2) const = 0;
        
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
