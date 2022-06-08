#include "../include/mrenv/Tesselation.h"

int main()
{
        // 0 - Parameters
        // 0.1 - Footprint
        int length = 1.5;
        int width = 1;
        // 0.2 - Map files that is in the maps/ folder
        std::string map_file = "map1.yaml";

        // 1 - Compute the polygon tesselaiton
        mrenv::Tesselation tessel;
        tessel.inputScenario(map_file, length, width);
        tessel.coverRectangles();
        tessel.plotBestCover();

        // 2 - Retrive a list of Rectangles
        auto rects  = tessel.getRectangles();

        return 0;
}

