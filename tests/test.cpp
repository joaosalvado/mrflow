//
// Created by ohmy on 2022-05-13.
//

#include "mrflow/cfree/SimpleCfree.h"
#include "mrenv/Tesselation.h"
int main(int argc, char** argv) {

    // 0 - Parameters
    double m2mm = 1000.0;
    // 0.1 - Footprint
    int length = 1.5; // meters
    int width = 1;    // meters
    // 0.2 - Map files that is in the maps/ folder
    std::string map_file = "map1.yaml";

    // 1.1 - Compute a rectangular tesselation of grayscale map
    mrenv::Tesselation tessel;
    tessel.setMapsPath("../maps/");
    tessel.inputScenario(map_file, length, width);
    tessel.coverRectangles(); // computes the tesselation
    tessel.plotBestCover();

    // 1.2 - Retrieve list of rectangles
    auto rects  = tessel.getRectangles();


    // 2 - Polygon Connectivity graph
    auto sCfree
        = std::make_shared<mrflow::cfree::SimpleCfree>(
                    length*m2mm, width*m2mm);
    // 2.1 - Add rectangles computed in mrenv
    sCfree->addMrenvPolygons(rects);
    // 2.2 - Create a graph expressing the connectivity between rectangles
    sCfree->createConnectivityGraph();
    sCfree->printMetaPolygons();

    sCfree->plotCfree();
    return 0;
}