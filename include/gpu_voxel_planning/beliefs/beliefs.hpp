#ifndef GVP_BELIEFS_HPP
#define GVP_BELIEFS_HPP
#include "gpu_voxel_rviz_visualization.hpp"

namespace GVP
{
    class Belief
    {
    public:
        virtual void viz(const GpuVoxelRvizVisualizer& viz) = 0;

        virtual double calcProbFree(const DenseGrid &volume) = 0;

        virtual double updateFreeSpace(const DenseGrid &new_free) = 0;

        virtual void updateCollisionSpace(Robot& robot, const DenseGrid &true_world) = 0;

        virtual DenseGrid sampleState() const = 0;

        virtual ~Belief() = default;
    };




    
    class ChsBelief : public Belief
    {
    public:
        std::vector<DenseGrid> chs;
        DenseGrid known_free;

    public:
        void viz(const GpuVoxelRvizVisualizer& viz) override
        {
            viz.vizChs(chs);
        }

        double calcProbFree(const DenseGrid &volume) override
        {
            PROFILE_START("CalcProbFreeCHS");
            double p_free = 1.0;
            for(auto &c: chs)
            {
                p_free *= 1 - ((double)c.collideWith(&volume)/c.countOccupied());
            }
            PROFILE_RECORD("CalcProbFreeCHS");
            return p_free;
        }

        double updateFreeSpace(const DenseGrid &new_free) override
        {
            known_free.add(&new_free);
            for(auto &c: chs)
            {
                c.subtract(&new_free);
            }
        }

        virtual void updateCollisionSpace(Robot& robot, const DenseGrid &true_world) override
        {
            addChs(robot, true_world);
        }
            
        virtual void addChs(Robot& robot, const DenseGrid &true_world)
        {
            auto link_occupancies = robot.getLinkOccupancies();
            size_t first_link_in_collision = 0;
            for(auto &link:link_occupancies)
            {
                if(link.overlapsWith(&true_world))
                {
                    break;
                }
                first_link_in_collision++;
            }

            if(first_link_in_collision >= link_occupancies.size())
            {
                throw std::logic_error("Trying to add CHS, but no link collided");
            }

            DenseGrid new_chs;

            for(size_t i=first_link_in_collision; i<link_occupancies.size(); i++)
            {
                new_chs.add(&link_occupancies[i]);
            }
            new_chs.subtract(&known_free);
            chs.push_back(new_chs);
        }

        virtual DenseGrid sampleState() const override
        {
            DenseGrid sampled_obstacles;
            for(const auto& c: chs)
            {
                c.copyRandomOccupiedElement(sampled_obstacles);
            }
            return sampled_obstacles;
        }
    };
}

#endif
