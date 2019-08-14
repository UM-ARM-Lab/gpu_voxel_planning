#include "strategies/victor_selective_densification.hpp"
#include "hacky_functions.hpp"


static const int DEFAULT_DEPTH = 18;
static const double DEFAULT_NUM_NEIGHBORS = 30.0;


static int numNodesAtDepth(int depth, int dim)
{
    int num_vert = std::pow(2, depth);
    return num_vert;
}

static double getConnectionRadius(int depth, int dim, double num_neighbors_desired)
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


std::vector<std::vector<double>> scaleToVictorDims(std::vector<std::vector<double>> points)
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




/****************************************
 **           SD Roadmap
 ***************************************/

SDRoadmap::SDRoadmap() : depth(5), dim(7), num_neighbors_desired(30.0)
{
    std::cout << "Generating SD roadmap\n";
    int seed = 0;
    generateGraph(depth, seed);
}

SDRoadmap::SDRoadmap(std::string filename, int seed) : depth(DEFAULT_DEPTH), dim(7),
                                                       num_neighbors_desired(DEFAULT_NUM_NEIGHBORS)
{
    if(loadFromFile(filename))
    {
        std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
        return;
    }

    std::cout << "Failed to load graph. Generating from scratch\n";
    generateGraph(depth, seed);
    std::cout << "Saving graph to " << filename << "\n";
    saveToFile(filename);
}


void SDRoadmap::generateGraph(int max_depth)
{
    throw std::runtime_error("Not implemented, use generateGraph with seed");
}

void SDRoadmap::generateGraph(int max_depth, int seed)
{
    std::cout << "generating...";
    std::cout << max_depth << ", " << dim << "\n";
    int num_vert = numNodesAtDepth(max_depth, dim);
    // int num_vert = 10;


    auto qs = scaleToVictorDims(halton::haltonPoints(num_vert, dim, seed));

    for(int depth=0; depth <= max_depth; depth++)
    {
        double radius = getConnectionRadius(depth, dim, num_neighbors_desired);
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

    double edge_radius = getConnectionRadius(depth, dim, num_neighbors_desired);
    auto inds_within_radius = getVerticesWithinRadius(new_node.toRaw(), edge_radius);


    int num_edges_added = 0;

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
                // std::cout << "New node (" << new_node_ind << ") and (" << near_ind
                //           << ") already connected by edge at higher level\n";
                          // << getEdge(above_ind, new_above_ind) << "\n";
                continue;
            }
        }


        num_edges_added++;        
        addEdgesBetweenNodes(new_node_ind, (int64_t)near_ind, edgeCost(new_node, getNodeValue(near_ind)));
    }
    std::cout << "Depth " << depth << ": num edges added with radius (" << edge_radius << ") is " << num_edges_added << "\n";

    return new_node_ind;
}

int64_t SDRoadmap::addVertexAndEdges(DepthNode dn)
{
    return addVertexAndEdges(dn.depth, dn.q);
}










/****************************************
 **           ID Roadmap
 ***************************************/
IDRoadmap::IDRoadmap(std::string filename, int seed) : depth(DEFAULT_DEPTH), dim(7),
                                                       num_neighbors_desired(DEFAULT_NUM_NEIGHBORS)
{
    if(loadFromFile(filename))
    {
        std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
        return;
    }

    std::cout << "Failed to load graph. Generating from scratch\n";
    generateGraph(depth, seed);
    std::cout << "Saving graph to " << filename << "\n";
    saveToFile(filename);
}


void IDRoadmap::generateGraph(int max_depth)
{
    throw std::runtime_error("Not implemented, use generateGraph with seed");
}

void IDRoadmap::generateGraph(int max_depth, int seed)
{
    std::cout << "generating...";
    std::cout << max_depth << ", " << dim << "\n";
    int num_vert = numNodesAtDepth(max_depth, dim);
    // int num_vert = 10;


    auto qs = scaleToVictorDims(halton::haltonPoints(num_vert, dim, seed));

    for(int depth=0; depth <= max_depth; depth++)
    {
        double radius = getConnectionRadius(depth, dim, num_neighbors_desired);
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


double IDRoadmap::edgeCost(const DepthNode &n1, const DepthNode &n2) const
{
    double d = EigenHelpers::Distance(n1.q, n2.q);
    return d;
}

double IDRoadmap::distanceHeuristic(const std::vector<double> &raw1,
                                   const std::vector<double> &raw2) const
{
    throw std::logic_error("This version of distance Heuristic should not be used");
}


int64_t IDRoadmap::addVertexAndEdges(int depth, std::vector<double> q)
{
    DepthNode new_node = DepthNode(depth, q);
    int64_t new_node_ind = addNode(new_node.toRaw());

    double edge_radius = getConnectionRadius(depth, dim, num_neighbors_desired);
    auto inds_within_radius = getVerticesWithinRadius(new_node.toRaw(), edge_radius);

    int num_edges_added = 0;

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

        num_edges_added++;        
        addEdgesBetweenNodes(new_node_ind, (int64_t)near_ind, edgeCost(new_node, getNodeValue(near_ind)));
    }
    std::cout << "Depth " << depth << ": num edges added with radius (" << edge_radius << ") is " << num_edges_added << "\n";

    return new_node_ind;
}

int64_t IDRoadmap::addVertexAndEdges(DepthNode dn)
{
    return addVertexAndEdges(dn.depth, dn.q);
}
