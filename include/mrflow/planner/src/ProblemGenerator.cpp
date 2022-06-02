//
// Created by ohmy on 2022-05-30.
//

#include "planner/ProblemGenerator.h"


using namespace mrflow::planner;

std::shared_ptr<RobotProblem>
ProblemGenerator::createRobotProblem(const std::vector<int> &robot_polygons){
    auto robot_problem = std::make_shared<RobotProblem>();
    // Gather sequence of polygons along robot path
    mrflow::cfree::Geometry::PolygonSet polygons_sequence;
    for(const int &p_id : robot_polygons){
        polygons_sequence.push_back( simpleCfree->getMetaPolygon(p_id) );
    }
    // Create a obstacle free space with obstacles from start to goal
    robot_problem->ofreebit = createOfreeBit( polygons_sequence);
    // Create a straight line path going through the sequence of polygons centers and intersecitons
    robot_problem->centerline = createCenterline(robot_polygons);

    return robot_problem;
}

std::shared_ptr<mrflow::cfree::OfreeBit>
ProblemGenerator::createOfreeBit(
        const std::vector<mrflow::cfree::Geometry::Polygon> &polygons){

    auto ofreebit = std::make_shared<cfree::OfreeBit>();
    // Union of all polygons
    mrflow::cfree::Geometry::Polygon union_polygons;
    for(auto polygon : polygons){
        simpleCfree->unionConvex(union_polygons, polygon, union_polygons);
    }
    auto convex_polygon = simpleCfree->convexhull(union_polygons);

    // Join convexhull with obstacle outter bounding boxes
    auto obstacles = simpleCfree->getObstaclesBBo();
    std::vector<std::shared_ptr<cfree::Geometry::Polygon>>  all = obstacles;
    all.push_back(std::make_shared<cfree::Geometry::Polygon>(convex_polygon));
    // Find the connectivity graph
    auto connectivity_graph
        = simpleCfree->connectivity_graph_general(all);

    auto connected_to_convexhull = connectivity_graph[connectivity_graph.size()-1];

    for(auto o_id : connected_to_convexhull){
        ofreebit->obstacles.push_back(simpleCfree->obstacles[o_id]);
    }
    ofreebit->convexhull = convex_polygon;

    // Debugging
    cv::Mat test_img = this->simpleCfree->getNewImage("map-partial-3.png"); // TODO: remove me
    this->simpleCfree->addFillPolygon(test_img, ofreebit->convexhull);
    for(auto obstacle : ofreebit->obstacles) {
        this->simpleCfree->addFillPolygon(test_img, *obstacle->bb_o);
    }
    cv::imshow("test", test_img);
    cv::waitKey();

    return ofreebit;
}


void ProblemGenerator::createMrPath(
        const std::vector<std::vector<int>> &mrpath){
    this->mrProblem_curr.clear(); // current multi-robot problem
    // Generate a problem per robot
    // Problem is composed of convex hull and obstacles (ofree) and a reference path
    for(const auto &robot_polygons : mrpath){
        this->mrProblem_curr.push_back( this->createRobotProblem(robot_polygons) );
    }
}

std::vector<mrflow::cfree::Geometry::Point>
ProblemGenerator::createCenterline(
        const std::vector<int> robot_polygons){
    std::vector<mrflow::cfree::Geometry::Point> path;
    int P = robot_polygons.size();
    // Center of first polygon
    const auto &first_polygon = simpleCfree->getMetaPolygon(robot_polygons[0]);
    path.push_back(simpleCfree->center(first_polygon));

    for(int p_curr  = 1; p_curr < P; ++p_curr){
        int p_prev = p_curr - 1;
        // center p_curr
        //simpleCfree->polygon_info_
        const auto &pol_curr = simpleCfree->getMetaPolygon(p_curr);
        // center p_prev
        const auto &pol_prev = simpleCfree->getMetaPolygon(p_curr);

    }
    return path;
}
