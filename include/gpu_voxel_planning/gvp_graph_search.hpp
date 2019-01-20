#ifndef LAZYSP_VOXELS_HPP
#define LAZYSP_VOXELS_HPP

#include "victor_halton_roadmap.hpp"
#include "state.hpp"
#include <graph_planner/dijkstras_addons.hpp>
#include <cmath>

namespace GVP
{
    typedef int64_t NodeIndex;

    // bool isValidFromKnownObs(std::vector<double> start, std::vector<double> end,
    //                          GpuVoxelsVictor* victor)
    // {
    //     auto path = interpolatePath(start, end, 0.1);
    //     victor->resetQuery();
    //     for(auto point: path)
    //     {
    //         victor->addQueryState(victor->toVictorConfig((point.data())));
    //     }
    //     return victor->countIntersect(VICTOR_QUERY_MAP, KNOWN_OBSTACLES_MAP) == 0;
    // }


    // bool checkEdge(arc_dijkstras::GraphEdge &e, arc_dijkstras::Graph<std::vector<double>> &g,
    //                GpuVoxelsVictor* victor)
    // {
    //     bool valid = isValidFromKnownObs(g.GetNodeImmutable(e.GetFromIndex()).GetValueImmutable(),
    //                                      g.GetNodeImmutable(e.GetToIndex()).GetValueImmutable(),
    //                                      victor);
    //     e.SetValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
    //                   arc_dijkstras::EDGE_VALIDITY::INVALID);
    //     return valid;
    // }


    // double calculateEdgeWeight(arc_dijkstras::Graph<std::vector<double>> &g,
    //                            arc_dijkstras::GraphEdge &e,
    //                            GpuVoxelsVictor* victor)
    // {
    //     PROFILE_START("calc_edge_weight");
    //     auto start = g.GetNodeMutable(e.GetFromIndex()).GetValueMutable();
    //     auto end = g.GetNodeMutable(e.GetToIndex()).GetValueMutable();
    //     auto path = interpolatePath(start, end, 0.1);
    //     victor->resetQuery();
    //     for(auto point: path)
    //     {
    //         victor->addQueryState(victor->toVictorConfig((point.data())));
    //     }

    //     auto overlaps = victor->countCHSCollisions();
    //     auto chs_sizes = victor->chsSizes();

    //     double p_no_col = 1.0;
    //     for(size_t i=0; i<overlaps.size(); i++)
    //     {
    //         p_no_col *= 1 - ((double)overlaps[i]) / (double)chs_sizes[i];
    //     }

    //     double alpha = 1.0;
    //     double chs_cost = -std::log(p_no_col);
    //     double l_cost = e.GetWeight();
    //     std::cout << "Calculating edge weight took: " << PROFILE_RECORD("calc_edge_weight") << "s\n";
    //     return l_cost + alpha * chs_cost;
    // }


    double evaluateEdge(arc_dijkstras::Graph<std::vector<double>> &g,
                        arc_dijkstras::GraphEdge &e,
                        const State &s)
    {
        // if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
        // {
        //     checkEdge(e, g, s);
        // }
        // return calculateEdgeWeight(g, e, s);
        return e.GetWeight();
    }




    std::vector<NodeIndex> planPath(NodeIndex start, NodeIndex goal, HaltonGraph &g, const State &s)
    {
        const auto eval_fn =
            [&s] (arc_dijkstras::Graph<std::vector<double>> &g, arc_dijkstras::GraphEdge &e)
            {
                return evaluateEdge(g, e, s);
            };

        auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformLazySP(
            g, start, goal, &distanceHeuristic, eval_fn, true);
        return result.first;
    }

}

#endif



