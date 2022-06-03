#ifndef MR_FLOW_PLANNER_H
#define MR_FLOW_PLANNER_H

#include "mrflow/base/MultirobotNetwork.hpp"
#include "mrflow/base/FlowRobots.hpp"
#include "mrflow/cfree/SimpleCfree.h"
#include "mrflow/planner/ProblemGenerator.h"
namespace mrflow::planner{
        enum optimize {SUM_TIME, MAX_TIME};
        class MrFlowPlanner
        {
        public:
            MrFlowPlanner( int numberRobots,  std::shared_ptr<mrflow::cfree::SimpleCfree> cfree_ = nullptr)
            {
                this->R = numberRobots;
                this->cfree = cfree_;
                this->P = this->cfree->getNumberOfMetaPolygons();
                double cost_stay_default = 0.1;
                this->cost_stay_ = std::vector<double>(this->P, cost_stay_default);
                this->optim = optimize::MAX_TIME;
                this->fr_ = std::make_shared<mrflow::base::FlowRobots>();
                this->setup();
            };

            bool solve(
                    std::vector<int> &start,
                    std::vector<int> &goal,
                    std::vector<std::vector<int>> &solution);
            bool solve_concatpath(
                    std::vector<int> &start,
                    std::vector<int> &goal,
                    std::vector<std::vector<int>> &solution);

            /**
             * @brief Generate the Dynamic network for the given environment (Cfree)
             * 
             */
            void setup();

            void setOptimizeMaxTime(){optim = optimize::MAX_TIME;} //default
            void setOptimizeSumTime(){optim = optimize::SUM_TIME;}


            void findTransition(
                    const std::vector<int> &x_prev,
                    const std::vector<int> &x_next,
                    int &P_in, int &P_out) const;
            std::vector<std::vector<int>> dummyLabelledPath(
                    std::vector<std::vector<int>> single_move_unlabeled_path);

            bool iterativeSolver(
                    std::vector<int> &start,
                    std::vector<int> &goal,
                    int Tmin, int Tmax);

        private:
            std::shared_ptr<mrflow::cfree::SimpleCfree> cfree;
            std::shared_ptr<mrflow::base::MultirobotNetwork> mrnet_;
            std::shared_ptr<mrflow::base::FlowRobots> fr_;
            std::vector<double> cost_stay_;
            optimize optim;
            int P; // Number of polygons
            int R; // Number of robots



//            ompl::base::State* toUnlabledState(
//                const std::vector<int> &abstract_state);
        };
} // namespace mrflow::planner

#endif //MR_FLOW_PLANNER_H