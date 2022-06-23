//
// Created by ohmy on 2022-06-10.
//

#include "mrflow/tessel/TesselationAdapte.h"
using namespace mrflow::tessel;

void TesselationAdapte::inputScenario(String map, double length, double width){
    this->mrenv->inputScenario(map, length, width);
}
void TesselationAdapte::coverPolygons(){
    this->mrenv->coverRectangles();
}

void TesselationAdapte::generateObstacles(){
    this->mrenv->generateObstacles();
}

std::vector<mrflow::cfree::Geometry::Polygon>
        TesselationAdapte::getPolygons() {
    std::vector<mrflow::cfree::Geometry::Polygon> return_polygons;
    auto rectangles = this->mrenv->getRectangles();
    for (auto &&rect: rectangles) {
        cfree::Geometry::Point lb = cfree::Geometry::createPoint(rect->left_bottom_corner.x - 2,
                                                                 rect->left_bottom_corner.y - 2);
        cfree::Geometry::Point lu = cfree::Geometry::createPoint(rect->left_bottom_corner.x - 2,
                                                                 rect->right_upper_corner.y + 2);
        cfree::Geometry::Point ru = cfree::Geometry::createPoint(rect->right_upper_corner.x + 2,
                                                                 rect->right_upper_corner.y + 2);
        cfree::Geometry::Point rb = cfree::Geometry::createPoint(rect->right_upper_corner.x + 2,
                                                                 rect->left_bottom_corner.y - 2);
        cfree::Geometry::Polygon poly =
                cfree::Geometry::createPolygon(
                        std::list<cfree::Geometry::Point>({lb, lu, ru, rb, lb}));
        return_polygons.push_back(poly);
    }
    return return_polygons;
}

std::vector<std::vector<cv::Point>>
TesselationAdapte::getObstacles(){
    return this->mrenv->getObstacles();
}


double TesselationAdapte::MtoPx(){
    return this->mrenv->MtoPx();
}
double TesselationAdapte::PxtoM(){
    return this->mrenv->PxtoM();
}
