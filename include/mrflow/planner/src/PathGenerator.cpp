//
// Created by ohmy on 2022-05-30.
//

#include "planner/PathGenerator.h"


using namespace mrflow::planner;

void PathGenerator::createPath( std::vector<int> robot_polygons){
    mrflow::cfree::Geometry::PolygonSet polygons_sequence;
    for(int p_id : robot_polygons){
        polygons_sequence.push_back( simpleCfree->getMetaPolygon(p_id) );
    }

    createOfreeBit(polygons_sequence[0], polygons_sequence[1]);
}

void PathGenerator::createOfreeBit(
        mrflow::cfree::Geometry::Polygon pol1,
        mrflow::cfree::Geometry::Polygon pol2){

    cv::Mat test_img = this->simpleCfree->getNewImage("map-partial-2.png"); // TODO: remove me
    this->simpleCfree->addFillPolygon(test_img, pol1);
    this->simpleCfree->addFillPolygon(test_img, pol2);
    cv::imshow("test", test_img);
    cv::waitKey();

    test_img = this->simpleCfree->getNewImage("map-partial-2.png"); // TODO: remove me
    mrflow::cfree::Geometry::Polygon pol_union;
    this->simpleCfree->unionConvex(pol1, pol2, pol_union);
    auto convexhull = this->simpleCfree->convexhull(pol_union);

    this->simpleCfree->addFillPolygon(test_img, convexhull);
    cv::imshow("test", test_img);
    cv::waitKey();

    test_img = this->simpleCfree->getNewImage("map-partial-2.png"); // TODO: remove me
    this->simpleCfree->addFillPolygon(test_img, pol_union);
    cv::imshow("test", test_img);
    cv::waitKey();


    test_img = this->simpleCfree->getNewImage("map-partial-2.png"); // TODO: remove me
    auto obstacles = this->simpleCfree->polygonMinus(convexhull,pol_union);
    for(auto obstacle : obstacles){
        this->simpleCfree->addFillPolygon(test_img, this->simpleCfree->createPolygon({obstacle}));}
    cv::imshow("test", test_img);
    cv::waitKey();

    //cv::ellipse(test_img,)

}

