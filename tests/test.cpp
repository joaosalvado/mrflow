//
// Created by ohmy on 2022-05-13.
//

#include "mrflow/cfree/SimpleCfree.h"
#include "mrenv/Tesselation.h"
#include "mrflow/planner/MrFlowPlanner.h"
#include <random>

bool random_mrstate(
        std::vector<int> &state, int R,
        const std::shared_ptr<mrflow::cfree::SimpleCfree> cfree);

int main(int argc, char** argv) {

    // 0 - Parameters
    int R = 10;
    double m2mm = 1000.0;
    // 0.1 - Footprint
    double length = 0.5; // meters
    double width = 0.5;    // meters
    // 0.2 - Map files that is in the maps/ folder
    std::string map_file = "map-partial-2";

    // 1.1 - Compute a rectangular tesselation of grayscale map
    mrenv::Tesselation tessel;
    tessel.setMapsPath("../maps/");
    tessel.inputScenario(map_file+".yaml", length, width);
    tessel.coverRectangles(); // computes the tesselation
    tessel.generateObstacles(); // computes bounding polygons of obstacles
     tessel.plotBestCover();
     tessel.plotObstaclesContour();

    // 1.2 - Retrieve list of rectangles and obstacles
    auto rects  = tessel.getRectangles();
    auto obstacles = tessel.getObstacles();

    // 2 - Polygon Connectivity graph (working with pixel metric)
    auto sCfree
        = std::make_shared<mrflow::cfree::SimpleCfree>(
                    length*tessel.MtoPx(),
                    width*tessel.MtoPx(),
                    tessel.PxtoM());
    // 2.1 - Add rectangles and obstacles computed in mrenv
    sCfree->addMrenvPolygons(rects);
    sCfree->addObstacles(obstacles);
    // 2.2 - Create a graph expressing the connectivity between rectangles
    sCfree->createConnectivityGraph();
    // sCfree->printMetaPolygons();
    sCfree->plotObstacles(map_file+".png");
    sCfree->plotCfree("cfree");


    // 3 - Sample start and goal
    std::vector<int> start, goal;
    random_mrstate(start, R,  sCfree);
    random_mrstate(goal, R, sCfree);
    // 4 - Multi-robot planner
    auto mrplanner = std::make_shared<mrflow::planner::MrFlowPlanner>(R,sCfree);
    std::vector<std::vector<int>> mrpath;
    mrplanner->solve_concatpath(start,goal, mrpath);

    sCfree->plotMultirobotPath(mrpath, map_file+".png");


    auto path_generator = std::make_shared<mrflow::planner::ProblemGenerator>(sCfree);
    auto mrpath_optim = path_generator->createMrPath(mrpath);

    int r = 0;
    for(auto robot_path : mrpath_optim){
        sCfree->loadMap(map_file + ".png");
        sCfree->plotPath(robot_path->ofreebit, robot_path->centerline, r++);
    }

    return 0;
}


bool random_mrstate(
        std::vector<int> &state, int R,
        const std::shared_ptr<mrflow::cfree::SimpleCfree> cfree){
    int P = cfree->getNumberOfMetaPolygons();
    const auto maxNumberRobots = cfree->polygon_info_.maxNumRobots;

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, P-1); // define the range
    state = std::vector<int>(P,0);
    while(R>0){
        // sample polygon
        int p_id = distr(gen);
        if(cfree->isPolygonDisconnected(p_id)) continue;
        // add if possible
        if(state[p_id]+1 <= maxNumberRobots[p_id]){
            state[p_id]++;
            R--;
        }
    }
    return true;
}
