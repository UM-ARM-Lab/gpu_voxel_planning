#include "strategies/victor_selective_densification.hpp"
#include "hacky_functions.hpp"


static int numNodesAtDepth(int depth, int dim)
{
    int num_vert = std::pow(2, depth);
    return num_vert;
}




SDRoadmap::SDRoadmap() : depth(5), dim(7), num_neighbors_desired(30.0)
{
    std::cout << "Generating SD roadmap\n";
    generateGraph(depth);
}

SDRoadmap::SDRoadmap(std::string filename) : depth(16), dim(7), num_neighbors_desired(30.0)
{
    if(loadFromFile(filename))
    {
        std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
        return;
    }

    std::cout << "Failed to load graph. Generating from scratch\n";
    generateGraph(depth);
    std::cout << "Saving graph to " << filename << "\n";
    saveToFile(filename);
}

// double SDRoadmap::radiusAtDepth(int depth)
// {
//     int num_vert = numNodesAtDepth(depth, dim);
//     double pow = -1.0 / (double)dim;
//     double multiplier = 7;
//     return multiplier * std::pow((double)num_vert, pow);
// }
double SDRoadmap::radiusAtDepth(int depth)
{
    if(dim != 7)
    {
        throw std::invalid_argument("radiusAtDepth only writted to accomodate dimension 7");
    }
    
    double num_points = (double)numNodesAtDepth(depth, dim);
    double space_volume = 1.0;

    for(int i=0; i<right_joint_lower_deg.size(); i++)
    {
        space_volume *= (right_joint_upper_deg[i] - right_joint_lower_deg[i])*torad;
    }

    double connection_ball_volume = (double)num_neighbors_desired * space_volume / num_points;
    double connection_radius = 0.8 * std::pow(connection_ball_volume, 1.0/dim);
    // https://en.wikipedia.org/wiki/Volume_of_an_n-ball

    return connection_radius;
}



void SDRoadmap::generateGraph(int max_depth)
{
    std::cout << "generating...";
    std::cout << max_depth << ", " << dim << "\n";
    int num_vert = numNodesAtDepth(max_depth, dim);
    // int num_vert = 10;


    auto qs = scaleToVictorDims(halton::haltonPoints(num_vert, dim));

    for(int depth=0; depth <= max_depth; depth++)
    {
        double radius = radiusAtDepth(depth);
        std::cout << "Adding nodes at depth " << depth << " with connection radius " << radius << "\n";
        for(int i=0; i<numNodesAtDepth(depth, dim); i++)
        {
            if(i%100000 == 0 && i != 0)
            {
                std::cout << "Added node " << i << "\n";
            }
            addVertexAndEdges(depth, qs[i]);
        }
        std::cout << "Graph has " << getNodes().size() << " nodes and " << countEdges() << " edges\n";
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
    throw std::logic_error("This version of distance Heuristic should not be used");
}




int64_t SDRoadmap::addVertexAndEdges(int depth, std::vector<double> q)
{
    DepthNode new_node = DepthNode(depth, q);
    int64_t new_node_ind = addNode(new_node.toRaw());
    int64_t above_ind = getNodeAt(depth - 1, q);
    if(above_ind >= 0)
    {
        auto &new_to_edge = addEdgeBetweenNodes(new_node_ind, above_ind,
                                                verticalEdgeCost(new_node,
                                                                 DepthNode(getNode(above_ind).getValue())));
        auto &new_from_edge = addEdgeBetweenNodes(above_ind, new_node_ind,
                                                  verticalEdgeCost(DepthNode(getNode(above_ind).getValue()),
                                                                   new_node));

        // addEdgesBetweenNodes(new_node_ind, above_ind,
        //                      verticalEdgeCost(new_node, DepthNode(getNode(above_ind).getValue())));
        new_to_edge.setValidity(arc_dijkstras::EDGE_VALIDITY::VALID);
        new_from_edge.setValidity(arc_dijkstras::EDGE_VALIDITY::VALID);

    }

    double edge_radius = radiusAtDepth(depth);
    auto inds_within_radius = getVerticesWithinRadius(new_node.toRaw(), edge_radius);

    for(const auto &near_ind:inds_within_radius)
    {
        if(new_node_ind == near_ind)
        {
            //Skip self-conenction edge
            continue;
        }
        if(new_node.depth != DepthNode(getNode(near_ind).getValue()).depth)
        {
            //Skip edges connecting nodes at different depths
            continue;
        }

        if(above_ind >= 0)
        {
            //Skip edges already accounted for in higher levels
            int64_t new_above_ind = getNodeAt(depth - 1, getNodeValue(near_ind).q);
            if(new_above_ind >= 0)
            {
                std::cout << "New node (" << new_node_ind << ") and (" << near_ind
                          << ") already connected by edge at higher level\n";
                          // << getEdge(above_ind, new_above_ind) << "\n";
                continue;
            }
        }
        
        addEdgesBetweenNodes(new_node_ind, (int64_t)near_ind, edgeCost(new_node, getNodeValue(near_ind)));
    }
    return new_node_ind;
}

int64_t SDRoadmap::addVertexAndEdges(DepthNode dn)
{
    return addVertexAndEdges(dn.depth, dn.q);
}

