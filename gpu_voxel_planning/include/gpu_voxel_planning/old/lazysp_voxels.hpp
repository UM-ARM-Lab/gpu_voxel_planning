#ifndef LAZYSP_VOXELS_HPP
#define LAZYSP_VOXELS_HPP

#include "victor_halton_roadmap.hpp"
#include "gpu_voxels_victor.hpp"
#include "helpers.hpp"
#include <arc_utilities/timing.hpp>
#include <graph_planner/dijkstras_addons.hpp>
#include <cmath>


bool isValidFromKnownObs(std::vector<double> start, std::vector<double> end,
                         GpuVoxelsVictor* victor)
{
    auto path = interpolatePath(start, end, 0.1);
    victor->resetQuery();
    for(auto point: path)
    {
        victor->addQueryState(victor->toVictorConfig((point.data())));
    }
    return victor->countIntersect(VICTOR_QUERY_MAP, KNOWN_OBSTACLES_MAP) == 0;
}


bool checkEdge(arc_dijkstras::GraphEdge &e, arc_dijkstras::Graph<std::vector<double>> &g,
               GpuVoxelsVictor* victor)
{
    bool valid = isValidFromKnownObs(g.GetNodeImmutable(e.GetFromIndex()).GetValueImmutable(),
                                     g.GetNodeImmutable(e.GetToIndex()).GetValueImmutable(),
                                     victor);
    e.SetValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                  arc_dijkstras::EDGE_VALIDITY::INVALID);
    return valid;
}


double calculateEdgeWeight(arc_dijkstras::Graph<std::vector<double>> &g,
                           arc_dijkstras::GraphEdge &e,
                           GpuVoxelsVictor* victor)
{
    PROFILE_START("calc_edge_weight");
    auto start = g.GetNodeMutable(e.GetFromIndex()).GetValueMutable();
    auto end = g.GetNodeMutable(e.GetToIndex()).GetValueMutable();
    auto path = interpolatePath(start, end, 0.1);
    victor->resetQuery();
    for(auto point: path)
    {
        victor->addQueryState(victor->toVictorConfig((point.data())));
    }

    auto overlaps = victor->countCHSCollisions();
    auto chs_sizes = victor->chsSizes();

    double p_no_col = 1.0;
    for(size_t i=0; i<overlaps.size(); i++)
    {
        p_no_col *= 1 - ((double)overlaps[i]) / (double)chs_sizes[i];
    }

    double alpha = 1.0;
    double chs_cost = -std::log(p_no_col);
    double l_cost = e.GetWeight();
    std::cout << "Calculating edge weight took: " << PROFILE_RECORD("calc_edge_weight") << "s\n";
    return l_cost + alpha * chs_cost;
}


double evaluateEdge(arc_dijkstras::Graph<std::vector<double>> &g,
                    arc_dijkstras::GraphEdge &e,
                    GpuVoxelsVictor* victor)
{
    if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
    {
        checkEdge(e, g, victor);
    }
    return calculateEdgeWeight(g, e, victor);
}




std::vector<int64_t> planPath(int start, int goal, HaltonGraph &g, GpuVoxelsVictor* victor)
{
    // for(auto e: g.E)
    // {
    //     checkEdge(e, g, victor);
    // }

    const auto eval_fn =
        [&victor] (arc_dijkstras::Graph<std::vector<double>> &g, arc_dijkstras::GraphEdge &e)
        {
            return evaluateEdge(g, e, victor);
        };

    auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformLazySP(
        g, start, goal, &distanceHeuristic, eval_fn, true);
    return result.first;
    
    // while(true)
    // {
    //     PROFILE_START("a_star");
    //     auto result = astar(start, goal, g, evaluatedEdges);
    //     auto path = result.first;
    //     if(path.size() == 0)
    //     {
    //         return path;
    //     }
        
    //     std::cout << "A_star in " << PROFILE_RECORD("a_star") << "s\n";

    //     auto validity_check_result = FORWARD_LAZY_CHECK_RESULT::EDGE_VALID;
    //     while(validity_check_result == FORWARD_LAZY_CHECK_RESULT::EDGE_VALID)
    //     {
    //         PROFILE_START("forward_check");
    //         validity_check_result = forwardLazyCheck(path, g, victor);
    //         std::cout << "Edge check in " << PROFILE_RECORD("forward_check") << "s\n";
    //         if(validity_check_result == FORWARD_LAZY_CHECK_RESULT::PATH_VALID)
    //         {
    //             return path;
    //         }
    //     }
    // }
}


// /***
//  *  Takes a single step on the path provided
//  *   Checks and updates edge validity
//  */
// int forwardMove(const std::vector<int64_t> &path, HaltonGraph &g, GpuVoxelsVictor* victor)
// {
//     arc_dijkstras::GraphEdge &e = g.GetNodeMutable(path[0]).GetEdgeMutable(path[1]);
//     assert(e.GetValidity() != arc_dijkstras::EDGE_VALIDITY::INVALID);
    
//     if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
//     {
//         checkEdge(e, g, victor);
//     }

//     int robot_location = path[0];
//     if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::VALID)
//     {
//         robot_location = path[1];
//     }
//     return robot_location;
// }



// enum FORWARD_LAZY_CHECK_RESULT {EDGE_INVALID, EDGE_VALID, PATH_VALID};

// /*
//  *  Checks the first unknown edge of $path$ in $g$ for collision
//  *   $path$ is a path of node indices through g
//  */
// FORWARD_LAZY_CHECK_RESULT
// forwardLazyCheck(const std::vector<int64_t> &path, HaltonGraph &g, GpuVoxelsVictor* victor)
// {
//     int i=0;
//     while(i < path.size() - 1)
//     {
//         arc_dijkstras::GraphEdge &e = g.GetNodeMutable(path[i]).GetEdgeMutable(path[i+1]);

//         if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::VALID)
//         {
//             i++;
//             continue;
//         }

//         if(checkEdge(e, g, victor))
//         {
//             return FORWARD_LAZY_CHECK_RESULT::EDGE_VALID;
//         }
//         return FORWARD_LAZY_CHECK_RESULT::EDGE_INVALID;
//         // std::cout << "Edge from point " << path[i] << " is " << 
//         //     (valid ? "valid" : "invalid") << "\n";
//         // std::cout << "Edge from point " << i << " is " << e.validity << "\n";
//     }
//     victor->resetQuery();
//     return FORWARD_LAZY_CHECK_RESULT::PATH_VALID;
// }



#endif



