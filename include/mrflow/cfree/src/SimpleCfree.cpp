

#include "mrflow/cfree/SimpleCfree.h"
#include <string>

using namespace mrflow;
void cfree::SimpleCfree::computePolygonsInfo()
{
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id)
    {
        //Initialize Poligon Info
        Polygon polygon = this->getMetaPolygon(p_id);
        this->polygon_info_.center.push_back(this->center(polygon));
        this->polygon_info_.maxNumRobots.push_back(this->maxNumberOfRobotsInPolygon(polygon));
    }
}

void cfree::SimpleCfree::setPolygonTransitionCosts()
{
    auto P = this->getNumberOfMetaPolygons();
    this->transition_cost_matrix_ = std::vector<std::vector<double>>(P, std::vector<double>(P, 0.0));
    for (auto p1 = 0; p1 < this->getNumberOfMetaPolygons(); ++p1)
    {
        for (auto p2 = 0; p2 < this->getNumberOfMetaPolygons(); ++p2)
        {
            if (p1 == p2)
            {
                transition_cost_matrix_[p1][p2] = 0.0;
            }
            else if (this->areMetaPolygonsConnected(p1, p2))
            {
                auto p1_center = this->center(this->getMetaPolygon(p1));
                auto p2_center = this->center(this->getMetaPolygon(p2));
                auto door_center = this->getCenterDoor(p1, p2);

                double euclidean_distance_Pin_Dcenter =
                    std::sqrt(std::pow(p1_center.x() - door_center.x(), 2) +
                              std::pow(p1_center.y() - door_center.y(), 2));

                double euclidean_distance_Dcenter_Pout =
                    std::sqrt(std::pow(door_center.x() - p2_center.x(), 2) +
                              std::pow(door_center.y() - p2_center.y(), 2));

                this->transition_cost_matrix_[p1][p2] = euclidean_distance_Pin_Dcenter + euclidean_distance_Dcenter_Pout;
            }
        }
    }
}

/*
void cfree::SimpleCfree::plotCfree()
{
    double f = mm_px; //0.01;
    int cfree_xmax = this->getCfreeMaxX() *1.05 * f;
    int cfree_ymax = this->getCfreeMaxY() *1.05* f;
    int cfree_xmin = this->getCfreeMinX() * f;
    int cfree_ymin = this->getCfreeMinY() * f;
    cv::Mat img = cv::Mat::zeros(cfree_ymax - cfree_ymin+1, cfree_xmax - cfree_xmin+1, CV_8UC3);
    //Plotting polygons with their respective number
    for (int pol_id = 0; pol_id < this->getNumberOfMetaPolygons(); ++pol_id)
    {
        const auto &pol = this->_metaPolygons[pol_id];
        cv::Point *points = new cv::Point[pol.coords_.size()];
        for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id)
        {
            points[point_id] = cv::Point2d(pol.coords_[point_id].x() * f, pol.coords_[point_id].y() * f);
        }
        fillPolygon(img, points, pol.coords_.size());
        const auto &pol_center = this->center(pol);
        writeText(
            img,
            cv::Point2d(pol_center.x() * f, pol_center.y() * f),
            std::string(std::to_string(pol_id)).c_str());
    }

    //Plotting the equivalent connectivity graph
    for (int p1 = 0; p1 < this->getNumberOfMetaPolygons(); ++p1)
    {
        for (int p2 = p1+1; p2 < this->getNumberOfMetaPolygons(); ++p2)
        {
            if( this->areMetaPolygonsConnected(p1, p2) ){
                const auto &p1_center = this->center( this->getMetaPolygon(p1) );
                cv::Point2d p1_center_cv = cv::Point2d(p1_center.x() * f, p1_center.y() * f);
                const auto &p2_center = this->center( this->getMetaPolygon(p2) );
                cv::Point2d p2_center_cv = cv::Point2d(p2_center.x() * f, p2_center.y() * f);
                this->line(img, p1_center_cv, p2_center_cv);
            }
        }
    }

    cv::imshow("Cfree", img);
    cv::waitKey();
}

void cfree::SimpleCfree::plotRectangles(PolygonSet &ps){
    double f = mm_px; //0.01;
    int cfree_xmax = this->getCfreeMaxX() * f;
    int cfree_ymax = this->getCfreeMaxY() * f;
    int cfree_xmin = this->getCfreeMinX() * f;
    int cfree_ymin = this->getCfreeMinY() * f;
    cv::Mat img = cv::Mat::zeros(cfree_ymax - cfree_ymin+1, cfree_xmax - cfree_xmin+1, CV_8UC3);
    //Plotting polygons with their respective number
    for (int pol_id = 0; pol_id < ps.size(); ++pol_id)
    {
        const auto &pol = ps.at(pol_id);
        cv::Point *points = new cv::Point[pol.coords_.size()];
        for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id)
        {
            points[point_id] = cv::Point2d(pol.coords_[point_id].x() * f, pol.coords_[point_id].y() * f);
        }
        fillPolygon(img, points, pol.coords_.size());
        const auto &pol_center = this->center(pol);
        writeText(
            img,
            cv::Point2d(pol_center.x() * f, pol_center.y() * f),
            std::string(std::to_string(pol_id)).c_str());
        cv::imshow("Polygon Set", img);
        cv::waitKey();
    }
//    cv::imshow("Polygon Set", img);
//    cv::waitKey();


}

void cfree::SimpleCfree::convexPolygon(cv::Mat img, const cv::Point *points, int n_pts)
{
    cv::fillPoly(img,
                 &points,
                 &n_pts,
                 1,
                 cv::Scalar(100, 100, 50));
}

void cfree::SimpleCfree::writeText(cv::Mat img, cv::Point point, const char *message)
{
    cv::putText(
        img,
        message,
        point,
        1,
        1,
        cv::Scalar(50, 255, 200));
}

void cfree::SimpleCfree::line(cv::Mat img, cv::Point start, cv::Point end)
{
    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::line(img,
             start,
             end,
             cv::Scalar(255, 0, 0),
             thickness,
             lineType);
}

void cfree::SimpleCfree::fillPolygon(Mat img, const cv::Point *points, int n_pts){
     fillPoly(img,
                 &points,
                 &n_pts,
                 1,
                 Scalar(50, 100, 50),
                 LINE_8);

        polylines(img,
                  &points,
                  &n_pts,
                  1,
                  true,
                  Scalar(240, 50, 200));
}*/


void cfree::SimpleCfree::createSquareCoverage(
    std::string maps_path,
    std::string map_file,
    double scale){
/*
    maps_path_ = maps_path; map_file_ = map_file;
    px_mm = scale;
    mm_px = 1/scale;
    if(this->footprint_==nullptr) std::cerr << "[sCfree] Footprint is not defined" << std::endl;
    tessel.setFootprint(this->footprint_->getLength()*mm_px, this->footprint_->getWidth()*mm_px);
    tessel.addPathToScenarios(maps_path);
    tessel.inputScenario(map_file);
    tessel.coverRectangles();

    auto &&rectangles = tessel.getRectangles();
    for(auto &&rect : rectangles){
        int f = 1000/10;
        Point lb =  this->createPoint(rect->left_bottom_corner.x*px_mm, rect->left_bottom_corner.y*px_mm);
        Point lu =  this->createPoint(rect->left_bottom_corner.x*px_mm, rect->right_upper_corner.y*px_mm);
        Point ru =  this->createPoint(rect->right_upper_corner.x*px_mm, rect->right_upper_corner.y*px_mm);
        Point rb =  this->createPoint(rect->right_upper_corner.x*px_mm, rect->left_bottom_corner.y*px_mm);
        Polygon poly = this->createPolygon(std::list<Point>({lb,lu,ru,rb,lb}));
        this->addPolygon(poly);
    }
*/

}