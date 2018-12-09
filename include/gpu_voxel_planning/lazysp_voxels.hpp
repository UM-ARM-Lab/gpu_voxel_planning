#ifndef LAZYSP_VOXELS_HPP
#define LAZYSP_VOXELS_HPP

#include "victor_halton_roadmap.hpp"
#include "gpu_voxels_victor.hpp"
#include "helpers.hpp"
#include <arc_utilities/timing.hpp>


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


bool checkEdge(Edge &e, Graph &g, GpuVoxelsVictor* victor)
{
    bool valid = isValidFromKnownObs(g.V[e.v1_ind].q, g.V[e.v2_ind].q, victor);
    e.validity = valid ? EDGE_VALIDITY::VALID : EDGE_VALIDITY::INVALID;
}


/*
 *  Checks the first unknown edge of $path$ in $g$ for collision
 *   $path$ is a path of node indices through g
 */
bool forwardLazyCheck(const std::vector<int> &path, Graph &g, GpuVoxelsVictor* victor)
{
    int i=0;
    while(i < path.size() - 1)
    {
        Edge &e = g.getEdge(path[i], path[i+1]);

        if(e.validity == EDGE_VALIDITY::VALID)
        {
            i++;
            continue;
        }

        checkEdge(e, g, victor);
        // std::cout << "Edge from point " << path[i] << " is " << 
        //     (valid ? "valid" : "invalid") << "\n";
        // std::cout << "Edge from point " << i << " is " << e.validity << "\n";
        return false;
    }
    victor->resetQuery();
    return true;
}




std::vector<int> planPath(int start, int goal, Graph &g, GpuVoxelsVictor* victor)
{
    // for(auto e: g.E)
    // {
    //     checkEdge(e, g, victor);
    // }
    
    while(true)
    {
        PROFILE_START("a_star");
        auto path = A_star(start, goal, g);
        if(path.size() == 0)
        {
            return path;
        }
        double dt = PROFILE_RECORD("a_star");
        std::cout << "A_star in " << dt << "\n";
        
        PROFILE_START("forward_check");
        if(forwardLazyCheck(path, g, victor))
        {
            return path;
        }
        dt = PROFILE_RECORD("forward_check");
        std::cout << "Edge check in " << dt << "\n";
    }
}


/***
 *  Takes a single step on the path provided
 *   Checks and updates edge validity
 */
int forwardMove(const std::vector<int> &path, Graph &g, GpuVoxelsVictor* victor)
{
    Edge &e = g.getEdge(path[0], path[1]);
    assert(e.validity != EDGE_VALIDITY::INVALID);
    
    if(e.validity == EDGE_VALIDITY::UNKNOWN)
    {
        checkEdge(e, g, victor);
    }

    int robot_location = path[0];
    if(e.validity == EDGE_VALIDITY::VALID)
    {
        robot_location = (path[0] == e.v1_ind) ? e.v2_ind : e.v1_ind;
    }
    return robot_location;
}





#endif



