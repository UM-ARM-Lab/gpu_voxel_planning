#ifndef VICTOR_SELECTIVE_DENSIFICATION_HPP
#define VICTOR_SELECTIVE_DENSIFICATION_HPP

#include <graph_planner/increasing_density_halton.hpp>




class SDRoadmap : public SelectiveDensificationGraph
{
public:
    int depth;
    int dim;
    double num_neighbors_desired;
    
    SDRoadmap();
    SDRoadmap(std::string filename, int seed=0);
    void generateGraph(int max_depth) override;
    void generateGraph(int max_depth, int seed);

    double edgeCost(const DepthNode &n1, const DepthNode &n2) const override;
    double distanceHeuristic(const std::vector<double> &raw1,
                             const std::vector<double> &raw2) const override;

    virtual int64_t addVertexAndEdges(int depth, std::vector<double> q) override;
    
    virtual int64_t addVertexAndEdges(DepthNode dn) override;
};


class IDRoadmap: public SelectiveDensificationGraph
{
public:
    int depth;
    int dim;
    double num_neighbors_desired;

    IDRoadmap(std::string filename, int seed=0);

    void generateGraph(int max_depth) override;
    void generateGraph(int max_depth, int seed);
    double edgeCost(const DepthNode &n1, const DepthNode &n2) const override;
    double distanceHeuristic(const std::vector<double> &raw1,
                             const std::vector<double> &raw2) const override;

    virtual int64_t addVertexAndEdges(int depth, std::vector<double> q) override;
    
    virtual int64_t addVertexAndEdges(DepthNode dn) override;

};

#endif
