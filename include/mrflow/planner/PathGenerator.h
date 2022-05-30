//
// Created by ohmy on 2022-05-30.
//

#ifndef MRFLOW_MRPATH_H
#define MRFLOW_MRPATH_H

#include <vector>
#include "mrflow/cfree/SimpleCfree.h"

namespace mrflow::planner {
    class PathGenerator {
        std::shared_ptr<mrflow::cfree::SimpleCfree> simpleCfree;
        std::vector<std::vector<int>> polygons_sequence;
        struct Ellipse {
            double a; // minor-axis
            double b; // major-axis
            double cx; // center x
            double cy; // center y
        };
        struct OfreeBit {
            mrflow::cfree::Geometry::Polygon convexhull;
            std::vector<Ellipse> obstacles;
        };

        std::vector<OfreeBit> ofree;
    public:
        PathGenerator(std::shared_ptr<mrflow::cfree::SimpleCfree> &simpleCfree_) {
            this->simpleCfree = simpleCfree_;
        }

        void createPath(std::vector<int> robot_polygons);
        void createOfreeBit(
                mrflow::cfree::Geometry::Polygon pol1,
                mrflow::cfree::Geometry::Polygon pol2);

    };
}


#endif //MRFLOW_MRPATH_H
