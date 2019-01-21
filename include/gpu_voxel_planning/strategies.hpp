#ifndef STRATEGIES_HPP
#define STRATEGIES_HPP

#include "victor_halton_roadmap.hpp"
#include "scenarios.hpp"
#include "path_utils_addons.hpp"

namespace GVP
{
    class Strategy
    {
    public:
        virtual GVP::Path applyTo(Scenario &scenario) = 0;
    };


}

#endif
