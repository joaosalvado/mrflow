

#include "mrflow/cfree/SimpleCfree.h"
#include <string>

using namespace mrflow;

void cfree::SimpleCfree::computePolygonsInfo() {
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id) {
        //Initialize Poligon Info
        Polygon polygon = this->getMetaPolygon(p_id);
        auto center_px = this->center(polygon);
        auto center_m = createPoint(center_px.x() * px2m, center_px.y() * px2m);
        this->polygon_info_.center.push_back(center_m);
        this->polygon_info_.maxNumRobots.push_back(this->maxNumberOfRobotsInPolygon(polygon));
    }
}

void cfree::SimpleCfree::setPolygonTransitionCosts() {
    auto P = this->getNumberOfMetaPolygons();
    this->transition_cost_matrix_ = std::vector<std::vector<double>>(P, std::vector<double>(P, 0.0));
    for (auto p1 = 0; p1 < this->getNumberOfMetaPolygons(); ++p1) {
        for (auto p2 = 0; p2 < this->getNumberOfMetaPolygons(); ++p2) {
            if (p1 == p2) {
                transition_cost_matrix_[p1][p2] = 0.0;
            //} else if (this->areMetaPolygonsConnected(p1, p2)) {
            } else if (this->m_door_center.find({p1, p2})
                            != this->m_door_center.end() ) {
            auto p1_center_px = this->center(this->getMetaPolygon(p1));
                auto p2_center_px = this->center(this->getMetaPolygon(p2));
                auto door_center_px = this->getCenterDoor(p1, p2);
                auto p1_center_m = createPoint(p1_center_px.x() * px2m, p1_center_px.y() * px2m);
                auto p2_center_m = createPoint(p2_center_px.x() * px2m, p2_center_px.y() * px2m);
                auto door_center_m = createPoint(door_center_px.x() * px2m, door_center_px.y() * px2m);

                double euclidean_distance_Pin_Dcenter =
                        std::sqrt(std::pow(p1_center_m.x() - door_center_m.x(), 2) +
                                  std::pow(p1_center_m.y() - door_center_m.y(), 2));

                double euclidean_distance_Dcenter_Pout =
                        std::sqrt(std::pow(door_center_m.x() - p2_center_m.x(), 2) +
                                  std::pow(door_center_m.y() - p2_center_m.y(), 2));

                this->transition_cost_matrix_[p1][p2] =
                        euclidean_distance_Pin_Dcenter + euclidean_distance_Dcenter_Pout;
            }
        }
    }
}


void cfree::SimpleCfree::plotCfree() {
    double f = mm_px; //0.01;
    int cfree_xmax = this->getCfreeMaxX() * 1.05 * f;
    int cfree_ymax = this->getCfreeMaxY() * 1.05 * f;
    int cfree_xmin = this->getCfreeMinX() * f;
    int cfree_ymin = this->getCfreeMinY() * f;


    cv::Mat img = cv::Mat::zeros(cfree_ymax - cfree_ymin + 1, cfree_xmax - cfree_xmin + 1, CV_8UC3);

    //Plotting polygons with their respective number
    for (int pol_id = 0; pol_id < this->getNumberOfMetaPolygons(); ++pol_id) {
        const auto &pol = this->_metaPolygons[pol_id];
        cv::Point *points = new cv::Point[pol.coords_.size()];
        for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id) {
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
    for (int p1 = 0; p1 < this->getNumberOfMetaPolygons(); ++p1) {
        for (int p2 = p1 + 1; p2 < this->getNumberOfMetaPolygons(); ++p2) {
            if (this->areMetaPolygonsConnected(p1, p2)) {
                const auto &p1_center = this->center(this->getMetaPolygon(p1));
                cv::Point2d p1_center_cv = cv::Point2d(p1_center.x() * f, p1_center.y() * f);
                const auto &p2_center = this->center(this->getMetaPolygon(p2));
                cv::Point2d p2_center_cv = cv::Point2d(p2_center.x() * f, p2_center.y() * f);
                this->line(img, p1_center_cv, p2_center_cv);
            }
        }
    }

    // Plot the obstcles
    for( auto obstacle : obstacles){
        cv::ellipse(img,{obstacle->center.x(), obstacle->center.y()},
                    {(int)obstacle->major_axis, (int)obstacle->minor_axis},
                    0,0,360,Scalar(50,50,50),4);
    }
    cv::imshow("Cfree", img);
    cv::waitKey();
}



void cfree::SimpleCfree::loadMap(String file){
    samples::addSamplesDataSearchPath("../../maps/");
    std::string image_path = samples::findFile(file);
    this->img = imread(image_path, IMREAD_COLOR);
}


void cfree::SimpleCfree::addFillPolygon(Mat img, Polygon pol){
    cv::Point *points = new cv::Point[pol.coords_.size()];
    for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id) {
        points[point_id] = cv::Point2d(pol.coords_[point_id].x() , pol.coords_[point_id].y());
    }
    fillPolygon(img, points, pol.coords_.size());
}

void cfree::SimpleCfree::plotPath(std::vector<int> path, int r) {
    // Plot the configuration freespace
    int f = 1;
    for (int pol_id : path) {
        const auto &pol = this->_metaPolygons[pol_id];
        cv::Point *points = new cv::Point[pol.coords_.size()];
        for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id) {
            points[point_id] = cv::Point2d(pol.coords_[point_id].x() * f, pol.coords_[point_id].y() * f);
        }
        fillPolygon(img, points, pol.coords_.size());
        const auto &pol_center = this->center(pol);
        writeText(
                img,
                cv::Point2d(pol_center.x() * f, pol_center.y() * f),
                std::string(std::to_string(pol_id)).c_str());
    }
    // Plot initial and final config
    // 1 ) Inital config
    auto polygons = path;
    // Start config polygon id
    int p_start_id = polygons[0];
    // poygon start config
    auto p_start = this->getMetaPolygon(p_start_id);
    // center start polygon
    const auto &p_start_center = this->center(p_start);
    // Convert ot cv::Point
    cv::Point2d p_start_center_cv = cv::Point2d(p_start_center.x(), p_start_center.y());
    // Plot circle
    //addFilledCircle(img, p_start_center_cv, r, this->footprint_->getLength() / 2);

// 2) Final config
// Final config polygon id
    int p_final_id = polygons[polygons.size() - 1];
// poygon final config
    auto p_final = this->getMetaPolygon(p_final_id);
// center final polygon
    const auto &p_final_center = this->center(p_final);
// Convert ot cv::Point
    cv::Point2d p_final_center_cv = cv::Point2d(p_final_center.x() + r, p_final_center.y() + r);
// Plot circle
    addFilledCircle(img, p_final_center_cv, r, this->footprint_->getLength() / 2);
    //addFilledCircle(img, p_final_center_cv, r, this->footprint_->getLength() / 4);


    // Plot path
    for (int p_seq_id = 0; p_seq_id < polygons.size() - 1; ++p_seq_id) {
    // polygon ids
        int p_prev_id = polygons[p_seq_id];
        int p_next_id = polygons[p_seq_id + 1];
    // polygon
        auto p_prev = this->getMetaPolygon(p_prev_id);
        auto p_next = this->getMetaPolygon(p_next_id);
    // centers
        const auto &p_prev_center = this->center(p_prev);
        cv::Point2d p_prev_center_cv = cv::Point2d(p_prev_center.x(), p_prev_center.y());
        const auto &p_next_center = this->center(p_next);
        cv::Point2d p_next_center_cv = cv::Point2d(p_next_center.x(), p_next_center.y());
    // door center
        auto door_center_px = this->getCenterDoor(p_prev_id, p_next_id);
        cv::Point2d door_center_cv = cv::Point2d(door_center_px.x(), door_center_px.y());

    // Plotting lines (path)
        this->line_color(img, p_prev_center_cv, door_center_cv, r);
        this->line_color(img, door_center_cv, p_next_center_cv, r);

    }



    cv::imshow("Cfree", img);
    cv::waitKey();
}

cv::Mat cfree::SimpleCfree::getNewImage(String file){
    samples::addSamplesDataSearchPath("../../maps/");
    std::string image_path = samples::findFile(file);
    return imread(image_path, IMREAD_COLOR);
}

void cfree::SimpleCfree::plotMultirobotPath(
        std::vector<std::vector<int>> path,
        String file) {
    // Plot the configuration freespace
    samples::addSamplesDataSearchPath("../../maps/");
    std::string image_path = samples::findFile(file);
    cv::Mat img = imread(image_path, IMREAD_COLOR);

    // Plot initial and final config
    int R = path.size();
    for (int r = 0; r < R; ++r) {
        // 1 ) Inital config
        auto polygons = path[r];
        // Start config polygon id
        int p_start_id = polygons[0];
        // poygon start config
        auto p_start = this->getMetaPolygon(p_start_id);
        // center start polygon
        const auto &p_start_center = this->center(p_start);
        // Convert ot cv::Point
        cv::Point2d p_start_center_cv = cv::Point2d(p_start_center.x() + r, p_start_center.y() + r);
        // Plot circle
        addFilledCircle(img, p_start_center_cv, r, this->footprint_->getLength() / 2);

        // 2) Final config
        // Final config polygon id
        int p_final_id = polygons[polygons.size() - 1];
        // poygon final config
        auto p_final = this->getMetaPolygon(p_final_id);
        // center final polygon
        const auto &p_final_center = this->center(p_final);
        // Convert ot cv::Point
        cv::Point2d p_final_center_cv = cv::Point2d(p_final_center.x() + r, p_final_center.y() + r);
        // Plot circle
        addFilledCircle(img, p_final_center_cv, r, this->footprint_->getLength() / 2);
        addFilledCircle(img, p_final_center_cv, R + 1, this->footprint_->getLength() / 4);
    }
    // Plot path
    for (int r = 0; r < R; ++r) {
        auto polygons = path[r];
        for (int p_seq_id = 0; p_seq_id < polygons.size() - 1; ++p_seq_id) {
            // polygon ids
            int p_prev_id = polygons[p_seq_id];
            int p_next_id = polygons[p_seq_id + 1];
            // polygon
            auto p_prev = this->getMetaPolygon(p_prev_id);
            auto p_next = this->getMetaPolygon(p_next_id);
            // centers
            const auto &p_prev_center = this->center(p_prev);
            cv::Point2d p_prev_center_cv = cv::Point2d(p_prev_center.x(), p_prev_center.y());
            const auto &p_next_center = this->center(p_next);
            cv::Point2d p_next_center_cv = cv::Point2d(p_next_center.x(), p_next_center.y());
            // door center
            auto door_center_px = this->getCenterDoor(p_prev_id, p_next_id);
            cv::Point2d door_center_cv = cv::Point2d(door_center_px.x(), door_center_px.y());

            // Plotting lines (path)
            this->line_color(img, p_prev_center_cv, door_center_cv, r);
            this->line_color(img, door_center_cv, p_next_center_cv, r);

        }
    }

    cv::imshow("Cfree", img);
    cv::waitKey();
}

void cfree::SimpleCfree::addFilledCircle(cv::Mat img, cv::Point center, int r, int radius) {
    cv::circle(img,
               center,
               radius,
               colors[r],
               FILLED,
               LINE_8);
}

/*
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
*/
void cfree::SimpleCfree::writeText(cv::Mat img, cv::Point point, const char *message) {
    cv::putText(
            img,
            message,
            point,
            1,
            1,
            cv::Scalar({50, 255, 200}));
}

void cfree::SimpleCfree::line(cv::Mat img, cv::Point start, cv::Point end) {
    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::line(img,
             start,
             end,
             cv::Scalar({255, 0, 0}),
             thickness,
             lineType);
}

void cfree::SimpleCfree::line_color(cv::Mat img, cv::Point start, cv::Point end, int r) {
    int thickness = 2;
    int lineType = cv::LINE_8;
    cv::line(img,
             start,
             end,
             colors[r],
             thickness,
             lineType);
}

void cfree::SimpleCfree::fillPolygon(Mat img, const cv::Point *points, int n_pts) {

   /*double alpha = 0.8;
    Mat layer = cv::Mat::zeros(img.size(), CV_8UC3);

    fillPoly(layer,
             &points,
             &n_pts,
             1,
             Scalar(50, 100, 50),
             LINE_8);

    cv::addWeighted(img, alpha, layer, 1-alpha, 0, img);*/


    polylines(img,
              &points,
              &n_pts,
              1,
              true,
              Scalar(70, 100, 70),
              LINE_8);
}


void cfree::SimpleCfree::addMrenvPolygons(
        std::list<std::shared_ptr<mrenv::Tesselation::Rectangle>> &rects) {
    for (auto &&rect: rects) {
        Geometry::Point lb = this->createPoint(rect->left_bottom_corner.x * px_mm - 2,
                                               rect->left_bottom_corner.y * px_mm - 2);
        Geometry::Point lu = this->createPoint(rect->left_bottom_corner.x * px_mm - 2,
                                               rect->right_upper_corner.y * px_mm + 2);
        Geometry::Point ru = this->createPoint(rect->right_upper_corner.x * px_mm + 2,
                                               rect->right_upper_corner.y * px_mm + 2);
        Geometry::Point rb = this->createPoint(rect->right_upper_corner.x * px_mm + 2,
                                               rect->left_bottom_corner.y * px_mm - 2);
        Geometry::Polygon poly = this->createPolygon(std::list<Geometry::Point>({lb, lu, ru, rb, lb}));
        this->addPolygon(poly);
    }
}



void cfree::SimpleCfree::addObstacles(std::vector<std::vector<cv::Point> > obstacles){
    PolygonSet obstacles_nonconvex;
    // Handle obstacles connected to the outter ring
    auto outter_ring = obstacles.front();
    std::list<Point> points;
    for(auto point : outter_ring){
        points.push_back({point.x, point.y});
    }
    points.push_back({outter_ring[0].x, outter_ring[0].y});
    auto outter_ring_pol = createPolygon(points);

    auto outter_ring_pol_convexhull = convexhull(outter_ring_pol);
    auto outter_ring_obstacles = polygonMinus(outter_ring_pol_convexhull,outter_ring_pol);
    for(auto obstacle : outter_ring_obstacles){obstacles_nonconvex.push_back(obstacle);}

    obstacles.erase(obstacles.begin()); // remove outter ring
    // Converting obstacles inside outter ring into polygons
    for(auto obstacle : obstacles){
        std::list<Point> points;
        for(auto point : obstacle){
            points.push_back({point.x, point.y});
        }
        points.push_back({obstacle[0].x, obstacle[0].y});
        obstacles_nonconvex.push_back(createPolygon(points));
    }
    // Split each obstacle
    for(auto obstacle_noncovex : obstacles_nonconvex){
        auto obstacles_convex = split(obstacle_noncovex);
        for(auto obstacle_convex : obstacles_convex){
            // Rectangle is the actual obstacle
            auto rectangle = gtl::view_as<gtl::rectangle_concept>(obstacle_convex);
            // Compute ellipse approx
            auto dx_ = boost::polygon::horizontal(rectangle);
            auto dx = dx_.high()-dx_.low();
            auto dy_ = boost::polygon::vertical(rectangle);
            auto dy = dy_.high()-dy_.low();
            Point center; boost::polygon::center(center, rectangle);
            double major_axis = dx/ sqrt(2);
            double minor_axis = dy/ sqrt(2);

            if(dx == 1 || dy ==1) continue; // TODO: odd ellipses
            // Creating outter bounding box
            Geometry::Point lb = this->createPoint(center.x() - major_axis,
                                                   center.y() - minor_axis);
            Geometry::Point lu = this->createPoint( center.x() - major_axis,
                                                    center.y() + minor_axis);
            Geometry::Point ru = this->createPoint(center.x() + major_axis,
                                                   center.y() + minor_axis);
            Geometry::Point rb = this->createPoint(center.x() + major_axis,
                                                   center.y() - minor_axis);
            Geometry::Polygon bb_out = this->createPolygon(std::list<Geometry::Point>({lb, lu, ru, rb, lb}));
            // Create obstacle
            this->obstacles.push_back(std::make_shared<mrflow::cfree::Obstacle>(
                    Obstacle{major_axis,minor_axis,center,
                             std::make_shared<Geometry::Polygon>(obstacle_convex),
                             std::make_shared<Geometry::Polygon>(bb_out)}));
        }
    }



}


void cfree::SimpleCfree::plotObstacles(String file){

    cv::Mat test_img = getNewImage(file); // TODO: remove me
    for( auto obstacle : obstacles){
        //addFillPolygon(test_img, obstacle->bb_i);
        addFillPolygon(test_img, *obstacle->bb_o);
        cv::ellipse(test_img,{obstacle->center.x(), obstacle->center.y()},
                {(int)obstacle->major_axis, (int)obstacle->minor_axis},
                0,0,360,Scalar(50,50,50),4);
    }
    cv::imshow("obstacles_ellipses", test_img);
    cv::waitKey();
}


std::vector<std::shared_ptr<cfree::Geometry::Polygon>>
cfree::SimpleCfree::getObstaclesBBo(){
    std::vector<std::shared_ptr<cfree::Geometry::Polygon>> output;
    for(auto obstacle : this->obstacles){
        output.push_back(obstacle->bb_o);
    }
    return output;
}

/*
void cfree::SimpleCfree::updateConnectivityGraphWithObstacles(){
    auto P = this->_metaConnectivityMap.size(); // amount of polygons
    // Generate ellipse obstacles boundind boxes
    PolygonSet  obstacles_bb;
    for(auto obstacle : obstacles) {
        Geometry::Point lb = this->createPoint(obstacle->center.x() - obstacle->major_axis,
                                               obstacle->center.y() - obstacle->minor_axis);
        Geometry::Point lu = this->createPoint( obstacle->center.x() - obstacle->major_axis,
                                                obstacle->center.y() + obstacle->major_axis);
        Geometry::Point ru = this->createPoint(obstacle->center.x() + obstacle->major_axis,
                                               obstacle->center.y() + obstacle->major_axis);
        Geometry::Point rb = this->createPoint(obstacle->center.x() + obstacle->major_axis,
                                               obstacle->center.y() - obstacle->major_axis);
        Geometry::Polygon poly = this->createPolygon(std::list<Geometry::Point>({lb, lu, ru, rb, lb}));
        obstacles_bb.push_back(poly);
    }


    for(int p1 = 0; p1 < P; ++p1){
         for(int p2 = 0; p2 < this->_metaConnectivityMap.size(); ++p2){
             auto ofreebit = std::make_shared<cfree::OfreePiece>();
            auto p1p2 = std::make_pair(p1,p2);
            // Compute Convex Hull
            auto pol1 = this->getMetaPolygon(p1); auto pol2 = this->getMetaPolygon(p2);
            mrflow::cfree::Geometry::Polygon pol1pol2_union;
            unionConvex(pol1, pol2, pol1pol2_union);
            auto pol1pol2_convexhull = this->convexhull(pol1pol2_union);
            //

         }
    }
}
*/
