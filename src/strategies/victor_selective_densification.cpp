#include "strategies/victor_selective_densification.hpp"
#include "hacky_functions.hpp"


static int numNodesAtDepth(int depth, int dim)
{
    int num_vert = std::pow(10, depth);
    return num_vert;
}

static double radiusAtDepth(int depth, int dim)
{
    int num_vert = numNodesAtDepth(depth, dim);
    double pow = -1.0 / (double)dim;
    double multiplier = 7;
    return multiplier * std::pow((double)num_vert, pow);
}



SDRoadmap::SDRoadmap() : depth(5), dim(7)
{
    std::cout << "Generating SD roadmap\n";
    generateGraph(depth);
}

SDRoadmap::SDRoadmap(std::string filename) : depth(5), dim(7)
{
    loadFromFile(filename);
    std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
}


void SDRoadmap::generateGraph(int max_depth)
{
    int dim=7;
    std::cout << "generating...";
    std::cout << max_depth << ", " << dim << "\n";
    int num_vert = numNodesAtDepth(max_depth, dim);
    // int num_vert = 10;


    auto qs = scaleToVictorDims(halton::haltonPoints(num_vert, dim));

    for(int depth=0; depth <= max_depth; depth++)
    {
        double radius = radiusAtDepth(depth, dim);
        std::cout << "Adding nodes at depth " << depth << " with connection radius " << radius << "\n";
        for(int i=0; i<numNodesAtDepth(depth, dim); i++)
        {
            addVertexAndEdges(depth, qs[i]);
        }
    }
}



std::vector<std::vector<double>> SDRoadmap::scaleToVictorDims(std::vector<std::vector<double>> points)
{
    std::cout << "Scaling to victor dims\n";
    for(auto &point: points)
    {
        for(int i=0; i<point.size(); i++)
        {
            double range = (right_joint_upper_deg[i] - right_joint_lower_deg[i]);
            point[i] = (point[i] * range + right_joint_lower_deg[i]) * torad;
        }
    }
    std::cout << "Scaled\n";
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




int64_t SDRoadmap::addVertexAndEdges(int depth, std::vector<double> q)
{
    DepthNode new_node = DepthNode(depth, q);
    int64_t new_node_ind = addNode(new_node.toRaw());
    int64_t above_ind = getNodeAt(depth - 1, q);
    if(above_ind >= 0)
    {
        addEdgesBetweenNodes(new_node_ind, above_ind,
                             verticalEdgeCost(new_node, DepthNode(getNode(above_ind).getValue())));
    }

    double edge_radius = radiusAtDepth(depth, dim);
    auto inds_within_radius = getVerticesWithinRadius(new_node.toRaw(), edge_radius);

    for(const auto &near_ind:inds_within_radius)
    {
        if(new_node_ind == near_ind)
        {
            continue;
        }
        if(new_node.depth != DepthNode(getNode(near_ind).getValue()).depth)
        {
            continue;
        }
        addEdgesBetweenNodes(new_node_ind, (int64_t)near_ind, edgeCost(new_node, getNodeValue(near_ind)));
    }
    return new_node_ind;
}

int64_t SDRoadmap::addVertexAndEdges(DepthNode dn)
{
    return addVertexAndEdges(dn.depth, dn.q);
}

