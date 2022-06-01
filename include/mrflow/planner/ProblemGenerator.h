//
// Created by ohmy on 2022-05-30.
//

#ifndef MRFLOW_MRPATH_H
#define MRFLOW_MRPATH_H

#include <vector>
#include "mrflow/cfree/SimpleCfree.h"

namespace mrflow::planner {
    struct RobotProblem{
        std::shared_ptr<mrflow::cfree::OfreeBit> ofreebit;
        std::vector<mrflow::cfree::Geometry::Point> centerline;
    };
    class ProblemGenerator {
        std::vector<std::shared_ptr<RobotProblem>> mrProblem_curr;
        std::shared_ptr<mrflow::cfree::SimpleCfree> simpleCfree;
        std::vector<std::vector<int>> polygons_sequence;

        std::shared_ptr<RobotProblem> createRobotProblem(const std::vector<int> &robot_polygons);

        std::shared_ptr<mrflow::cfree::OfreeBit> createOfreeBit(const
                std::vector<mrflow::cfree::Geometry::Polygon> &polygons);

    public:
        ProblemGenerator(std::shared_ptr<mrflow::cfree::SimpleCfree> &simpleCfree_) {
            this->simpleCfree = simpleCfree_;
        }

        void createMrPath(const std::vector<std::vector<int>> &mrpath);

    };
}


#endif //MRFLOW_MRPATH_H
