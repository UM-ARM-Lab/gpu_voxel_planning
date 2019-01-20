#include "victor_halton_roadmap.hpp"
#include "hacky_functions.hpp"
#include <arc_utilities/timing.hpp>




Roadmap::Roadmap():
    HaltonGraph(0,0,0)
{
    // int num_vert = 1000;
    // edge_dist = 4.0;
    // int num_vert = 10000;
    // r_disc = 2.0;
    // int num_vert = 100000;
    // r_disc = 1.5;
    int num_vert = 500000;
    r_disc = 1.0;

    auto configs = scaleToVictorDims(halton::haltonPoints(num_vert, 7));

    PROFILE_START("node_creation");
    int i=0;
    for(auto q:configs)
    {
        addVertexAndEdges(q);
        i++;
        if(i%10000 == 0)
        {
            std::cout << i << "/" << num_vert << " nodes in " <<
                PROFILE_RECORD("node_creation") << "s\n";
        }
    }
    // V = toNodes(haltonPoints(bases, num_vert, offsets));
    // addEdges(edge_dist);

    std::cout << "Made graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
}


Roadmap::Roadmap(std::string filename):
    HaltonGraph(0,0,0)
{
    loadFromFile(filename);

    std::cout << "Loaded graph with " << nodes_.size() << " vertices and " << countEdges() << " edges\n";
    
}


std::vector<std::vector<double>> Roadmap::scaleToVictorDims(std::vector<std::vector<double>> points)
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
