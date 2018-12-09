#include "victor_halton_roadmap.hpp"
#include "hacky_functions.hpp"




Roadmap::Roadmap(GpuVoxelsVictor* victor):
    victor_(victor)
{
    // int num_vert = 1000;
    // edge_dist = 4.0;
    // int num_vert = 10000;
    // edge_dist = 2.0;
    int num_vert = 100000;
    edge_dist = 1.5;
    
    std::vector<int> bases{2,3,5,7,11,13,17};
    std::vector<int> offsets{100, 120, 234, 182, 102, 192, 476};
    // auto nodes = toNodes(scaleToVictorDims(halton::haltonPoints(num_vert, 7)));
    auto nodes = toNodes(scaleToVictorDims(halton::haltonPoints(bases, num_vert, offsets)));

    for(auto n:nodes)
    {
        insertVertex(n);
    }
    // V = toNodes(haltonPoints(bases, num_vert, offsets));
    // addEdges(edge_dist);

    std::cout << "Made graph with " << V.size() << " vertices and " << E.size() << " edges\n";
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
