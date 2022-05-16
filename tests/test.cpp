//
// Created by ohmy on 2022-05-13.
//

//#include "mrflow/cfree/SimpleCfree.h"
#include "mrenv/Tesselation.h"
int main(int argc, char** argv) {

    // 0 - Parameters
    // 0.1 - Footprint
    int length = 1.5; // meters
    int width = 1;    // meters
    // 0.2 - Map files that is in the maps/ folder
    std::string map_file = "map1.yaml";

    // 1 - Compute a rectangular tesselation of grayscale map
    mrenv::Tesselation tessel;
    tessel.inputScenario(map_file, length, width);
    tessel.coverRectangles(); // computes the tesselation
    tessel.plotBestCover();

    // 2 - Retrieve list of rectangles
    auto rects  = tessel.getRectangles();


    return 0;
}