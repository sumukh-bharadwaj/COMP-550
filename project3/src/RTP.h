///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Hemanth Kumar Jayakumar, Koride Sumukh Bharadwaj
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H
#include "ompl/control/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        // TODO: Implement RTP as described

        class RTP : public base::Planner
        {
            public:
                RTP(const base::SpaceInformationPtr &si);
                ~RTP() override;

                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
                
                void clear() override;
                
                void getPlannerData(base::PlannerData &data) const override;

                // base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        };

    }  // namespace geometric
}  // namespace ompl

#endif
