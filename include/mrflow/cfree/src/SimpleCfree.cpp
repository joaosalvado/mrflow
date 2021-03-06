

#include "mrflow/cfree/SimpleCfree.h"
#include <string>

using namespace mrflow;

void cfree::SimpleCfree::computePolygonsInfo() {
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id) {
        //Initialize Poligon Info
        Polygon polygon = this->getMetaPolygon(p_id);
        auto center_px = this->center(polygon);
        auto center_m = createPoint(center_px.x() , center_px.y() );
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
                       != this->m_door_center.end()) {
                auto p1_center_px = this->center(this->getMetaPolygon(p1));
                auto p2_center_px = this->center(this->getMetaPolygon(p2));
                auto door_center_px = this->getCenterDoor(p1, p2);
                auto p1_center_m = createPoint(p1_center_px.x() , p1_center_px.y() );
                auto p2_center_m = createPoint(p2_center_px.x() , p2_center_px.y() );
                auto door_center_m = createPoint(door_center_px.x() , door_center_px.y() );

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


void cfree::SimpleCfree::plotCfree(String title) {
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
                const auto &door = this->getCenterDoor(p1,p2);
                cv::Point2d door_cv = cv::Point2d(door.x() * f, door.y() * f);
                this->line(img, p1_center_cv, door_cv);
                this->line(img, p2_center_cv, door_cv);
            }
        }
    }

    // Plot the obstcles
    for (auto obstacle: obstacles) {
        cv::ellipse(img, {(int)obstacle->xc, (int)obstacle->yc},
                    {(int) obstacle->major_axis, (int) obstacle->minor_axis},
                    0, 0, 360, Scalar(50, 50, 50), 4);
    }
    cv::imshow(title, img);
    cv::waitKey();
}


void cfree::SimpleCfree::loadMap(String file) {
    samples::addSamplesDataSearchPath("../../maps/");
    std::string image_path = samples::findFile(file);
    this->img = imread(image_path, IMREAD_COLOR);
}


void cfree::SimpleCfree::addFillPolygon(Mat img, Polygon pol) {
    cv::Point *points = new cv::Point[pol.coords_.size()];
    for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id) {
        points[point_id] = cv::Point2d(pol.coords_[point_id].x(), pol.coords_[point_id].y());
    }
    fillPolygon(img, points, pol.coords_.size());
}


void cfree::SimpleCfree::addFillPolygon(Mat img, Polygon pol, int color_id){
    cv::Point *points = new cv::Point[pol.coords_.size()];
    for (auto point_id = 0; point_id < pol.coords_.size(); ++point_id) {
        points[point_id] = cv::Point2d(pol.coords_[point_id].x(), pol.coords_[point_id].y());
    }
    fillPolygon(img, points, pol.coords_.size(), color_id);
}

void cfree::SimpleCfree::plotPath(std::vector<int> path, int r) {
    // Plot the configuration freespace
    int f = 1;
    for (int pol_id: path) {
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

cv::Mat cfree::SimpleCfree::getNewImage(String file) {
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
              LINE_4);
}

void cfree::SimpleCfree::fillPolygon(Mat img, const cv::Point *points, int n_pts, int color_id)  {
    polylines(img,
              &points,
              &n_pts,
              1,
              true,
              colors[color_id],
              LINE_4);
}




void cfree::SimpleCfree::addMrenvPolygons(
        std::vector<Geometry::Polygon> &polygons) {
    for (auto &poly: polygons) {
        this->addPolygon(poly);
    }
}


void cfree::SimpleCfree::addObstacles(std::vector<std::vector<cv::Point> > obstacles) {
    PolygonSet obstacles_nonconvex;
    // Handle obstacles connected to the outter ring
    auto outter_ring = obstacles.front();
    std::list<Point> points;
    for (auto point: outter_ring) {
        points.push_back({point.x, point.y});
    }
    points.push_back({outter_ring[0].x, outter_ring[0].y});
    auto outter_ring_pol = createPolygon(points);

    auto outter_ring_pol_convexhull = convexhull(outter_ring_pol);
    auto outter_ring_obstacles = polygonMinus(outter_ring_pol_convexhull, outter_ring_pol);
    for (auto obstacle: outter_ring_obstacles) { obstacles_nonconvex.push_back(obstacle); }

    obstacles.erase(obstacles.begin()); // remove outter ring
    // Converting obstacles inside outter ring into polygons
    for (auto obstacle: obstacles) {
        std::list<Point> points;
        for (auto point: obstacle) {
            points.push_back({point.x, point.y});
        }
        points.push_back({obstacle[0].x, obstacle[0].y});
        obstacles_nonconvex.push_back(createPolygon(points));
    }
    // Split each obstacle
    for (auto obstacle_noncovex: obstacles_nonconvex) {
        auto obstacles_convex = split(obstacle_noncovex);
        for (auto obstacle_convex: obstacles_convex) {
            // Rectangle is the actual obstacle
            auto rectangle = gtl::view_as<gtl::rectangle_concept>(obstacle_convex);
            // Compute ellipse approx
            auto dx_ = boost::polygon::horizontal(rectangle);
            auto dx = dx_.high() - dx_.low();
            auto dy_ = boost::polygon::vertical(rectangle);
            auto dy = dy_.high() - dy_.low();
            Point center;
            boost::polygon::center(center, rectangle);
            double major_axis = dx / sqrt(2);
            double minor_axis = dy / sqrt(2);

            if (dx == 1 || dy == 1) continue; // TODO: odd ellipses
            // Creating outter bounding box
            Geometry::Point lb = this->createPoint(center.x() - major_axis,
                                                   center.y() - minor_axis);
            Geometry::Point lu = this->createPoint(center.x() - major_axis,
                                                   center.y() + minor_axis);
            Geometry::Point ru = this->createPoint(center.x() + major_axis,
                                                   center.y() + minor_axis);
            Geometry::Point rb = this->createPoint(center.x() + major_axis,
                                                   center.y() - minor_axis);
            Geometry::Polygon bb_out = this->createPolygon(std::list<Geometry::Point>({lb, lu, ru, rb, lb}));
            // Create obstacle
            this->obstacles.push_back(std::make_shared<mrflow::cfree::Obstacle>(
                    Obstacle{major_axis, minor_axis, (double)center.x(), (double)center.y(),
                             std::make_shared<Geometry::Polygon>(obstacle_convex),
                             std::make_shared<Geometry::Polygon>(bb_out)}));
        }
    }


}


void cfree::SimpleCfree::plotObstacles(String file) {

    cv::Mat test_img = getNewImage(file); // TODO: remove me
    for (auto obstacle: obstacles) {
        //addFillPolygon(test_img, obstacle->bb_i);
        addFillPolygon(test_img, *obstacle->bb_o);
        cv::ellipse(test_img, {(int)obstacle->xc, (int)obstacle->yc},
                    {(int) obstacle->major_axis, (int) obstacle->minor_axis},
                    0, 0, 360, Scalar(50, 50, 50), 4);
    }
    cv::imshow("obstacles_ellipses", test_img);
    cv::waitKey();
}


std::vector<std::shared_ptr<cfree::Geometry::Polygon>>
cfree::SimpleCfree::getObstaclesBBo() {
    std::vector<std::shared_ptr<cfree::Geometry::Polygon>> output;
    for (auto obstacle: this->obstacles) {
        output.push_back(obstacle->bb_o);
    }
    return output;
}

void cfree::SimpleCfree::updateConnectivityGraphWithObstacles() {
    auto P = this->_metaConnectivityMap.size(); // amount of polygons
    // Generate ellipse obstacles boundind boxes
    auto obstacles_bb = this->getObstaclesBBo();

    for (int p_id = 0; p_id < P; ++p_id) {
        // Remove obstacle bounding boxes and check if robot fits in polygon
        Polygon pol_no_obstacles;
        if(!robotFitsPolygon(obstacles_bb, getMetaPolygon(p_id), pol_no_obstacles)){
            // Remove it
            //this->_metaPolygons.erase(this->_metaPolygons.begin()+p_id);
            this->_metaConnectivityMap[p_id].clear();
            for(int p_other = 0; p_other < P; ++p_other){
                this->eraseConnection(p_other,p_id);
            }
        }
        // After removing bb, check if the doors are still traversable
        for(auto p_other : this->_metaConnectivityMap[p_id]){
            if(p_other == p_id) continue;
            // Check if robot can traverse between polygons after removing obstacles
            if(!arePolygonsNoObstaclesConnected(this->getMetaPolygon(p_id),pol_no_obstacles, this->getMetaPolygon(p_other))){
                this->eraseConnection(p_id,p_other);
                //std::remove(this->_metaConnectivityMap[p_id].begin(), this->_metaConnectivityMap[p_id].end(),p_other);
                //std::remove(this->_metaConnectivityMap[p_other].begin(), this->_metaConnectivityMap[p_other].end(),p_id);
                 cv::Mat test_img = getNewImage("map-partial-3.png"); // TODO: remove me
                 addFillPolygon(test_img, pol_no_obstacles);
                 addFillPolygon(test_img, this->getMetaPolygon(p_other));
                 cv::imshow("test", test_img);
                 cv::waitKey();
            }
        }
    }
}

bool cfree::SimpleCfree::robotFitsPolygon(
        const std::vector<std::shared_ptr<Polygon>> &obstacles,
        Polygon polygon, Polygon &polygon_no_obstacles){
    // Gather obstacles and polygon
    auto all_polygons = obstacles;
    all_polygons.push_back(std::make_shared<Polygon>(polygon));

    // Find the connectivity graph
    auto connectivity_graph
            = connectivity_graph_general(all_polygons);

    auto connected_to_polygon = connectivity_graph[connectivity_graph.size()-1];

    //cv::Mat test_img = getNewImage("map-corridors.png"); // TODO: remove me
    polygon_no_obstacles = polygon;
    bool broken = false;
   // addFillPolygon(test_img, polygon_no_obstacles, 3);
    for(auto o_id : connected_to_polygon){

        auto result_set = polygonMinus(polygon_no_obstacles, *this->obstacles[o_id]->bb_o);
        if(!result_set.empty()){
            polygon_no_obstacles = result_set.front();
        }

        if(result_set.size() != 1) {
            broken = true; // obstacle divided/crossed polygon
        }
/*        addFillPolygon(test_img, *this->obstacles[o_id]->bb_o, 1);
        addFillPolygon(test_img, polygon_no_obstacles, 5);
        cv::imshow("test", test_img);
        cv::waitKey();*/
    }

/*   addFillPolygon(test_img, polygon_no_obstacles, 2);
    cv::imshow("test", test_img);
    cv::waitKey();*/
    if(broken) return false;
    auto area = this->area(polygon_no_obstacles);
    auto L =  margin*footprint_->getLength();
    auto min_area = 4*margin*L*L;
    if( area < 4*margin*L*L ) return false;
    return true;
}


bool cfree::SimpleCfree::arePolygonsNoObstaclesConnected(
        const Polygon &pol_original,
        const Polygon &pol_no_obstacles,
        const Polygon &pol_connected){
    PolygonSet p_o{pol_original}, p_c{pol_connected}, p_no{pol_original};
    // Intersection pol_original and pol_connected
    PolygonSet p_o_c;
    assign(p_o_c, p_o & p_c);
    // Intersection pol_no_obstacles and pol_connected
    PolygonSet p_no_c;
    assign(p_no_c, p_no & p_c);
    // Intersection minus
    auto obstacle_piece = polygonMinus(p_o_c.front(), p_no_c.front());

    // Compute original door length
    auto p_o_c_rect = gtl::view_as<gtl::rectangle_concept>(p_o_c.front());
    auto dx_interval = boost::polygon::horizontal(p_o_c_rect);
    auto dx = dx_interval.high() - dx_interval.low();
    auto dy_interval = boost::polygon::vertical(p_o_c_rect);
    auto dy = dy_interval.high() - dy_interval.low();
    auto orginal_door_length = (dx > dy ? dx : dy);

    int door_length;
    if(obstacle_piece.empty()){ // obstcacle does not overlap the door
        door_length = orginal_door_length;
    } else{ // re compute the  door without obstacle
        // Compute obstacle door length
        auto obs_rect = gtl::view_as<gtl::rectangle_concept>(obstacle_piece.front());
        auto dx_interval = boost::polygon::horizontal(obs_rect);
        auto dx = dx_interval.high() - dx_interval.low();
        auto dy_interval = boost::polygon::vertical(obs_rect);
        auto dy = dy_interval.high() - dy_interval.low();
        auto obs_door_length = (dx > dy ? dx : dy);

        door_length = orginal_door_length - obs_door_length;
    }

    if(door_length < margin*this->footprint_->getLength()  ) {
        return false;
    }
    return true;
}

void cfree::SimpleCfree::plotPath(
        std::shared_ptr<OfreeBit> ofreebit_meters,
        std::vector<Point> centerline,  int r){
    int cfree_xmax = this->getCfreeMaxX() * 1.05;
    int cfree_ymax = this->getCfreeMaxY() * 1.05;
    int cfree_xmin = this->getCfreeMinX();
    int cfree_ymin = this->getCfreeMinY();
    // cv::Mat img_path = cv::Mat::zeros(cfree_ymax - cfree_ymin + 1, cfree_xmax - cfree_xmin + 1, CV_8UC3);
    // Plot convex hull
    const auto &convexhull = ofreebit_meters->convexhull;
    addFillPolygon(img, convexhull);
    // Plot obstacles as ellipses
    for (auto obstacle: ofreebit_meters->obstacles) {
        auto x = static_cast<int>(obstacle->xc);
        auto y = static_cast<int>(obstacle->yc);
        auto a = static_cast<int>(obstacle->major_axis);
        auto b = static_cast<int>(obstacle->minor_axis);
        cv::ellipse(img, {x, y},
                    { a, b},
                    0, 0, 360, Scalar(50, 200, 50), 4);
    }
    // Plot center line
    for( int path_pt_id = 0; path_pt_id < centerline.size()-1; ++path_pt_id ){
        const auto &point_prev = centerline[path_pt_id];
        const auto &point_next = centerline[path_pt_id+1];
        cv::Point2d point_prev_cv = cv::Point2d(point_prev.x(), point_prev.y());
        cv::Point2d point_next_cv = cv::Point2d(point_next.x(), point_next.y());
        this->line_color(img, point_prev_cv, point_next_cv, r);
    }

    cv::imshow("Path",img);
    cv::waitKey();
}

// TODO: implementation ongoing
void cfree::SimpleCfree::samplePolygon(int pol_id, double &x, double &y){
    // get the polygon
    auto polygon = this->getMetaPolygon(pol_id);
    auto rectangle = gtl::view_as<gtl::rectangle_concept>(polygon);
    // get center
    auto center = this->polygon_info_.center[pol_id];
    // get min side
    auto dx_interval = gtl::horizontal(rectangle);
    auto dx = dx_interval.high() - dx_interval.low();
    auto dy_interval = gtl::vertical(rectangle);
    auto dy = dy_interval.high() - dy_interval.low();
    auto min_side = (dx < dy ? dx : dy);
    // random device
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, min_side - footprint_->getLength()); // define the range

    //
    auto radius = distr(gen);


}

std::vector<int> cfree::SimpleCfree::fromXYtoPolygons(
        std::vector<std::vector<double>> mrstate_meters){
    int P = this->getNumberOfMetaPolygons();
    int R = mrstate_meters.size();
    auto polygons_state_return = std::vector<int>(P, 0);

    for(int r = 0; r < R; ++r ){
        const auto &x = mrstate_meters[r][0] * this->m2px;
        const auto &y = mrstate_meters[r][1] * this->m2px;
        for(int p = 0; p < P; ++p){
            if(this->isInPolygon(p, x, y)){
                polygons_state_return[p]++;
                continue;
            }
        }
    }
    return polygons_state_return;
}
