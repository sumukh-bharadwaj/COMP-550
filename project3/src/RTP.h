///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Hemanth Kumar Jayakumar, Koride Sumukh Bharadwaj
//////////////////////////////////////

#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

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

                void freeMemory();
                
                void setup() override;

                /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
                class Motion
                {
                public:
                    Motion() = default;

                    /** \brief Constructor that allocates memory for the state */
                    Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                    {
                    }

                    ~Motion() = default;

                    /** \brief The state contained by the motion */
                    base::State *state{nullptr};

                    /** \brief The parent motion in the exploration tree */
                    Motion *parent{nullptr};
                };

                /** \brief Free the memory allocated by this planner */
                // void freeMemory();

                /** \brief Compute distance between motions (actually distance between contained states) */
                // double distanceFunction(const Motion *a, const Motion *b) const
                // {
                //     return si_->distance(a->state, b->state);
                // }

                /** \brief State sampler */
                base::StateSamplerPtr sampler_;

                /** \brief A nearest-neighbors datastructure containing the tree of motions */
                // std::shared_ptr<NearestNeighbors<Motion *>> nn_;

                /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
                 * available) */
                double goalBias_{.05};

                /** \brief The maximum length of a motion to be added to a tree */
                // double maxDistance_{0.};

                /** \brief The random number generator */
                RNG rng_;

                /** \brief The most recent goal motion.  Used for PlannerData computation */
                Motion *lastGoalMotion_{nullptr};

                /** \brief A nearest-neighbors datastructure containing the tree of motions */
                /* Nearest-neightbor search is not conducted here and only the tree is being used */
                std::shared_ptr<NearestNeighbors<Motion *>> nn_;
                
                double maxDistance_{0.};
                
                double distanceFunction(const Motion *a, const Motion *b) const
                {
                    return si_->distance(a->state, b->state);
                }
        };

    }  // namespace geometric
}  // namespace ompl

#endif
