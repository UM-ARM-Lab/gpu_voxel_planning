#include "victor_selective_densification.hpp"
#include "hacky_functions.hpp"


int numNodesAtDepth(int depth, int dim)
{
    int vert_per_edge = std::pow(2, depth) + 1;
    int num_vert = std::pow(vert_per_edge, dim);
    return num_vert;
}



SDRoadmap::SDRoadmap()
{
    int max_depth = 6;
    generateGraph(max_depth);
}

SDRoadmap::SDRoadmap(std::string filename)
{
    loadFromFile(filename);
    std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
}


void SDRoadmap::generateGraph(int max_depth)
{
    int dim=7;
    int num_vert = numNodesAtDepth(max_depth, dim);
    
    auto qs = scaleToVictorDims(halton::haltonPoints(num_vert, dim));

    for(int depth=0; depth <= max_depth; depth++)
    {
        for(int i=0; i<numNodesAtDepth(depth, dim); i++)
        {
            addVertexAndEdges(depth, qs[i]);
        }
    }
}



std::vector<std::vector<double>> SDRoadmap::scaleToVictorDims(std::vector<std::vector<double>> points)
{
    for(auto &point: points)
    {
        for(int i=0; i<point.size(); i++)
        {
            double range = (right_joint_upper_deg[i] - right_joint_lower_deg[i]);
            point[i] = (point[i] * range + right_joint_lower_deg[i]) * torad;
        }
    }
    return points;
}


double SDRoadmap::edgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    double d = EigenHelpers::Distance(n1.q, n2.q);
    return d;
}

double SDRoadmap::distanceHeuristic(const std::vector<double> &raw1,
                                   const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    // std::cout << "Calling dist heuristic with depth " << d1.depth << "\n";
     // std::pow(2, d1.depth);
    return EigenHelpers::Distance(d1.q, d2.q)*std::pow(1.5, d1.depth);
}

