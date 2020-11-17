#ifndef GPU_VICTOR_HELPERS_HPP
#define GPU_VICTOR_HELPERS_HPP

#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>
#include <arc_utilities/eigen_helpers.hpp>

double pCollision(std::vector<size_t> chs_overlaps, std::vector<size_t> chs_sizes)
{
    double p_no_col = 1.0;
    assert(chs_overlaps.size() == chs_sizes.size());

    for(size_t i=0; i < chs_overlaps.size(); i++)
    {
        assert(chs_overlaps[i] <= chs_sizes[i]);
        double p_col_i = (double)chs_overlaps[i] / (double)chs_sizes[i];
        
        p_no_col *= (1.0 - p_col_i);
    }

    return 1.0 - p_no_col;
}

double IG(std::vector<size_t> chs_overlaps, std::vector<size_t> chs_sizes)
{
    double ig = 0;
    assert(chs_overlaps.size() == chs_sizes.size());

    for(size_t i=0; i < chs_overlaps.size(); i++)
    {
        assert(chs_overlaps[i] <= chs_sizes[i]);
        double p_col_i = (double)chs_overlaps[i] / (double)chs_sizes[i];

        ig -= std::log2(1-p_col_i);
    }

    return ig;
}




/***
 *   Returns a path from p1 to p2 with points interpolated with distance d
 */
std::vector<std::vector<double>> interpolatePath(const std::vector<double>& v1,
                                                 const std::vector<double>& v2,
                                                 double d)
{
    std::vector<std::vector<double>> path;
    double total_dist = EigenHelpers::Distance(v1, v2);
    int num_points = (int)(total_dist/d) + 1;

    std::vector<double> dx;
    dx.resize(v1.size());
    for(int i=0; i<v1.size(); i++)
    {
        dx[i] = (v2[i] - v1[i]) / num_points;
    }

    for(int i=0; i<=num_points; i++)
    {
        std::vector<double> p = v1;
        for(int coord=0; coord<v1.size(); coord++)
        {
            p[coord] += dx[coord]*i;
        }
        path.push_back(p);
    }
    return path;    
}



/***
 *   Pauses exeuction until key is pressed
 */
void waitForKeypress()
{
    std::string unused;
    std::cout << "Waiting for user input...\n";
    std::getline(std::cin, unused);

}


#endif
