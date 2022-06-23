//
// Created by ohmy on 2022-06-10.
//

#ifndef MROPT_TESSELATIONADAPTE_H
#define MROPT_TESSELATIONADAPTE_H

#include "mrflow/tessel/TesselInterface.h"
#include "mrenv/Tesselation.h"

namespace mrflow::tessel {
    class TesselationAdapte : public TesselInterface {
    public:
        std::shared_ptr<mrenv::Tesselation> mrenv;

        TesselationAdapte() {
            this->mrenv = std::make_shared<mrenv::Tesselation>();
        }
         ~TesselationAdapte() = default;

        void inputScenario(String map, double length, double width) override;

        void coverPolygons() override; // computes the tesselation
        void generateObstacles() override; // computes bounding polygons of obstacles

        std::vector<mrflow::cfree::Geometry::Polygon> getPolygons() override;
        std::vector<std::vector<cv::Point>> getObstacles() override;

        double MtoPx() override;
        double PxtoM() override;

        void plotBestCover(){this->mrenv->plotBestCover();}
        void plotObstaclesContour(){this->mrenv->plotObstaclesContour();}
    };
}


#endif //MROPT_TESSELATIONADAPTE_H
