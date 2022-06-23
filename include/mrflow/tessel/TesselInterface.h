//
// Created by ohmy on 2022-06-10.
//

#ifndef MROPT_TESSELINTERFACE_H
#define MROPT_TESSELINTERFACE_H

#include "mrflow/cfree/SimpleCfree.h"

namespace mrflow::tessel {
    class TesselInterface {
    public:
        TesselInterface() = default;
        virtual ~TesselInterface() = default;

        virtual void inputScenario(String map, double length, double width) = 0;

        virtual void coverPolygons() = 0; // computes the tesselation
        virtual void generateObstacles() = 0; // computes bounding polygons of obstacles

        virtual std::vector<mrflow::cfree::Geometry::Polygon> getPolygons() = 0;
        virtual std::vector<std::vector<cv::Point>> getObstacles() = 0;

        virtual double MtoPx(){ return 1;}
        virtual double PxtoM(){ return 1;}

    };
}

#endif //MROPT_TESSELINTERFACE_H
