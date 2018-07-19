#ifndef GPU_VICTOR_HELPERS_HPP
#define GPU_VICTOR_HELPERS_HPP

#include <vector>
#include <cassert>
#include <iostream>

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



void waitForKeypress()
{
    std::string unused;
    std::cout << "Waiting for user input to start...\n";
    std::getline(std::cin, unused);

}


#endif
