#ifndef VICTOR_SELECTIVE_DENSIFICATION_HPP
#define VICTOR_SELECTIVE_DENSIFICATION_HPP

#include <graph_planner/increasing_density_halton.hpp>




class SDRoadmap : public SelectiveDensificationGraph
{
public:
    int depth;
    int dim;
    
    SDRoadmap();
    SDRoadmap(std::string filename);
    std::vector<std::vector<double>> scaleToVictorDims(std::vector<std::vector<double>> points);
    void generateGraph(int max_depth) override;

    double edgeCost(const DepthNode &n1, const DepthNode &n2) const override;
    // double distanceHeuristic(const std::vector<double> &raw1,
    //                          const std::vector<double> &raw2) const override;
    double distanceHeuristic(const std::vector<double> &raw1,
                             const std::vector<double> &raw2) const override;

    virtual int64_t addVertexAndEdges(int depth, std::vector<double> q) override;
    
    virtual int64_t addVertexAndEdges(DepthNode dn) override;

};

#endif

