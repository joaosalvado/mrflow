//
// Created by ohmy on 2020-10-27.
//

#include "mrflow/cfree/Geometry.h"

using namespace mrflow::cfree;

Geometry::Point Geometry::createPoint(int x, int y)
{
    Point p(x, y);
    return p;
}

Geometry::Polygon Geometry::createPolygon(std::list<Point> &listPoints)
{
    Polygon poly;
    poly.set(listPoints.begin(), listPoints.end());
    return poly;
}
Geometry::Polygon Geometry::createPolygon(std::list<Point> &&listPoints)
{
    Polygon poly;
    poly.set(listPoints.begin(), listPoints.end());
    return poly;
}

Geometry::Polygon Geometry::polygonsIntersection(const Polygon &poly1, const Polygon &poly2)
{
    PolygonSet ps1{poly1}, ps2{poly2};

    //intersectionPolygon += gtl::view_as<gtl::polygon_90_concept>(poly1) & gtl::view_as<gtl::polygon_90_concept>(poly2);
    //p1 &= p2;
    PolygonSet ps3;
    assign(ps3, ps1 & ps2);
    if(ps3.empty()){
        return createPolygon({});
    } else {
        //Since polygons will not have inner rings / holes and are convex then the intersection is always ONE polygon
        return ps3.front();
    }
}

Geometry::Polygon Geometry::polygonsUnion(const Polygon &poly1, const Polygon &poly2)
{
    PolygonSet ps1{poly1}, ps2{poly2};

    //intersectionPolygon += gtl::view_as<gtl::polygon_90_concept>(poly1) & gtl::view_as<gtl::polygon_90_concept>(poly2);
    //p1 &= p2;
    PolygonSet ps3, ps4, ps5, ps6, ps7;

    assign(ps3, ps1 ^ ps2);
    //Dealing with adjoint polygons, since there might be 3 co-linear points
    Polygon p_union = ps3.front();
    std::vector<int> unnecessaryPoints{};
    //Checking which points are 3 points are colinear and save the middle point
    for (int pt = 0; pt < p_union.size() - 1; ++pt)
    {
        auto point_1 = p_union.coords_.at(pt);
        auto point_2 = p_union.coords_.at(pt + 1);
        Point point_3;
        if (pt == p_union.size() - 2)
        {
            point_3 = p_union.coords_.at(1);
        }
        else
        {
            point_3 = p_union.coords_.at(pt + 2);
        }

        if (Geometry::angle(point_1, point_2) == Geometry::angle(point_1, point_3))
        {
            unnecessaryPoints.push_back(pt + 1);
        }
    }

    //Create a new polgygon without the colinear points
    std::list<Point> listPoints;
    for (int pt = 0; pt < p_union.size(); ++pt)
    {
        if (std::find(unnecessaryPoints.begin(), unnecessaryPoints.end(), pt) == unnecessaryPoints.end())
        {
            listPoints.push_back(std::move(p_union.coords_.at(pt)));
        }
    }

    Polygon poly = Geometry::createPolygon(listPoints);

    //Since polygons will not have inner rings / holes and are convex then the intersection is always ONE polygon
    return poly;
}

//PoligonSet

Geometry::Polygon Geometry::polygonSetIntersection(const PolygonSet &Polygons)
{
    if (Polygons.size() == 1)
    {
        return Polygons.at(0);
    }
    else
    {
        Polygon intersectionPolygons = Polygons.at(0);

        for (int p_i = 1; p_i < Polygons.size(); ++p_i)
        {
            intersectionPolygons = polygonsIntersection(intersectionPolygons, Polygons.at(p_i));
        }

        return intersectionPolygons;
    }
}

Bounds Geometry::boundaries(const Polygon &poly)
{
    Bounds xyBounds;
    xyBounds.xMin = poly.coords_[0].x();
    xyBounds.xMax = poly.coords_[0].x();
    xyBounds.yMin = poly.coords_[0].y();
    xyBounds.yMax = poly.coords_[0].y();

    for (int i = 1; i < poly.size(); ++i)
    {
        if (xyBounds.xMin > poly.coords_[i].x())
            xyBounds.xMin = poly.coords_[i].x();
        if (xyBounds.xMax < poly.coords_[i].x())
            xyBounds.xMax = poly.coords_[i].x();
        if (xyBounds.yMin > poly.coords_[i].y())
            xyBounds.yMin = poly.coords_[i].y();
        if (xyBounds.yMax < poly.coords_[i].y())
            xyBounds.yMax = poly.coords_[i].y();
    }

    return xyBounds;
}

Geometry::Line_g Geometry::perpendicularLine(std::vector<Point_g> &line_segment)
{
    auto x_0 = line_segment.at(0).get<0>();
    auto y_0 = line_segment.at(0).get<1>();
    auto x_1 = line_segment.at(1).get<0>();
    auto y_1 = line_segment.at(1).get<1>();

    Point other_point;
    auto middle_point = Geometry::createPoint((x_0 + x_1) * 0.5, (y_0 + y_1) * 0.5);
    if (x_0 == x_1)
    { //line_segment is vertical
        other_point = Geometry::createPoint(middle_point.x() + 1000, middle_point.y());
    }
    else
    { //line_segment is Horizontal
        other_point = Geometry::createPoint(middle_point.x(), middle_point.y() + 1000);
    }

    return std::move(Geometry::createLine(middle_point, other_point));
}

bool Geometry::areIntersecting(const Polygon &poly1, const Polygon &poly2)
{
    PolygonSet p1{poly1}, p2{poly2};
    PolygonSet ps3;
    assign(ps3, p1 & p2);
    //Since polygons will not have inner rings / holes and are convex then the intersection is always ONE polygon
    return gtl::area(ps3.front()) > 0;
}

std::list<std::pair<int, int>> Geometry::convertPolygon(Polygon poly)
{
    std::list<std::pair<int, int>> newPolygon{};
    for (auto coord : poly.coords_)
    {
        newPolygon.push_back(std::pair<int, int>(coord.x(), coord.y()));
    }

    return newPolygon;
}

int Geometry::maximumSide(const Polygon &poly)
{
    std::size_t numPoints = poly.size();
    Point p_start, p_end;
    int maxDistance{0};

    for (int p_i = 0; p_i < numPoints; ++p_i)
    {
        p_start = poly.coords_.at(p_i);
        if (p_i == numPoints - 1)
        {
            p_end = poly.coords_.at(0);
        }
        else
        {
            p_end = poly.coords_.at(p_i + 1);
        }
        auto distance = gtl::euclidean_distance(p_start, p_end);
        if (maxDistance < distance)
            maxDistance = distance;
    }

    return maxDistance;
}

Geometry::Point Geometry::center(const Polygon &poly)
{
    Point center{};
    gtl::center(center, poly);
    return center;
}

/**
 * Computes the area of Polygon poly
 * @param poly
 * @return
 */
double Geometry::area(const Polygon &poly)
{
    double area = gtl::area(poly);
    return area;
}

//x-coordinate min for a rectangle polygon clockwise written
int Geometry::xMin(const Polygon &poly)
{
    const Point &bottomLeftCorner = poly.coords_.at(0);
    return Geometry::getX(bottomLeftCorner);
}
//x-coordinate max for a rectangle polygon clockwise written
int Geometry::xMax(const Polygon &poly)
{
    const Point &topRightCorner = poly.coords_.at(2);
    return Geometry::getX(topRightCorner);
}
//y-coordinate min for a rectangle polygon clockwise written
int Geometry::yMin(const Polygon &poly)
{
    const Point &bottomLeftCorner = poly.coords_.at(0);
    return Geometry::getY(bottomLeftCorner);
}
//y-coordinate max for a rectangle polygon clockwise written
int Geometry::yMax(const Polygon &poly)
{
    const Point &topRightCorner = poly.coords_.at(2);
    return Geometry::getY(topRightCorner);
}

double Geometry::angle(const Point &p1, const Point &p2)
{
    return std::atan2(Geometry::getY(p2) - Geometry::getY(p1), Geometry::getX(p2) - Geometry::getX(p1));
}

Geometry::Polygon_g Geometry::convertPolygonGeometric(const Polygon &poly)
{
    Polygon_g new_polygon;
    for (auto point : poly.coords_)
    {
        boost::geometry::append(new_polygon.outer(), Point_g(point.x(), point.y()));
    }

    return std::move(new_polygon);
}

bool Geometry::isConvex(const Polygon &poly)
{

    bool output = false;

    Polygon_g hull;
    Polygon_g poly_g = Geometry::convertPolygonGeometric(poly);
    boost::geometry::convex_hull(poly_g, hull);

    auto area_h = boost::geometry::area(hull);
    auto area_p = boost::geometry::area(poly_g);

    if (area_p == area_h)
    {
        output = true;
    }

    return output;
}

Geometry::Polygon Geometry::convertGeometricPolygon(const Polygon_g &poly)
{
    std::list<Point> listPoints{};
    for (auto point_it = bg::points_begin(poly); point_it != bg::points_end(poly); ++point_it)
    {
        double x = bg::get<0>(*point_it);
        double y = bg::get<1>(*point_it);
        listPoints.push_back(Geometry::createPoint(x, y));
    }
    //listPoints.reverse();
    Polygon p = Geometry::createPolygon(listPoints);

    auto a = boost::polygon::area(p);
    auto b = boost::geometry::area(poly);

    return std::move(p);
}

bool Geometry::unionConvex(const Polygon &p1, const Polygon &p2, Polygon &union_p1_p2)
{
    Polygon_g p1_g = Geometry::convertPolygonGeometric(p1);
    Polygon_g p2_g = Geometry::convertPolygonGeometric(p2);

    std::vector<Polygon_g> output;
    boost::geometry::union_(p1_g, p2_g, output);
    if (output.size() > 1)
    {
        return false;
    }

    for (auto point_it = bg::points_begin(output.at(0)); point_it != bg::points_end(output.at(0)); ++point_it)
    {
        if (point_it == bg::points_begin(output.at(0)))
        {
            continue;
        }
        //Swap
        auto prev_point = --point_it;
        ++point_it;
        auto next_point = point_it;
        auto temp_point = prev_point;

        bg::set<0>(*prev_point, bg::get<0>(*next_point));
        bg::set<1>(*prev_point, bg::get<1>(*next_point));
        bg::set<0>(*next_point, bg::get<0>(*prev_point));
        bg::set<1>(*next_point, bg::get<1>(*prev_point));
    }

    auto start_point = bg::points_begin(output.at(0));
    auto end_point = bg::points_end(output.at(0));
    --end_point;
    bg::set<0>(*end_point, bg::get<0>(*start_point));
    bg::set<1>(*end_point, bg::get<1>(*start_point));

    //Removing redundant vertecis
    Polygon_g simplified;
    boost::geometry::simplify(output.at(0), simplified, 0.0001);

    union_p1_p2 = std::move(Geometry::convertGeometricPolygon(simplified));

    if (!Geometry::isConvex(union_p1_p2))
    {
        return false;
    }
    else
    {
        return true;
    }
}

std::vector<Geometry::Point_g> Geometry::intersectingLineSegment(const Polygon &P1, const Polygon &P2)
{
    auto P1_g = Geometry::convertPolygonGeometric(P1);
    auto P2_g = Geometry::convertPolygonGeometric(P2);

    std::vector<Point_g> line_segment;
    boost::geometry::intersection(P1_g, P2_g, line_segment);

    return line_segment;
}

bool Geometry::isLineSegmentHorizontal(const std::vector<Point_g> &line_segment)
{
    const Point_g &Point_1 = line_segment.at(0);
    const Point_g &Point_2 = line_segment.at(1);
    if (Point_1.get<0>() == Point_2.get<0>())
        return false;

    return true;
}

void Geometry::expandAdjacentPolygonsMaximalOverlap(Polygon &poly1, Polygon &poly2)
{
    //Compute Intersecting Line Segment
    auto line_seg = Geometry::intersectingLineSegment(poly1, poly2);
    for(auto &point : line_seg){ //Deal with rounding errors
        point.set<0>((int) point.get<0>());
        point.set<1>((int) point.get<1>());
    }

    if (line_seg.size() == 0)
        return;
    const Point_g &point_1 = line_seg.at(0);
    const Point_g &point_2 = line_seg.at(1);

    int x_min, x_max, y_min, y_max;

    if (Geometry::isLineSegmentHorizontal(line_seg))
    {
        //x-coordinate is fixed and evaluate
        auto x1 = point_1.get<0>();
        auto x2 = point_2.get<0>();
        if (x1 < x2)
        {
            x_min = x1;
            x_max = x2;
        }
        else
        {
            x_min = x2;
            x_max = x1;
        }

        auto poly1_y_max = Geometry::yMax(poly1);
        auto poly2_y_max = Geometry::yMax(poly2);
        if (poly1_y_max > poly2_y_max)
        {
            y_max = poly1_y_max;
        }
        else
        {
            y_max = poly2_y_max;
        }

        auto poly1_y_min = Geometry::yMin(poly1);
        auto poly2_y_min = Geometry::yMin(poly2);
        if (poly1_y_min < poly2_y_min)
        {
            y_min = poly1_y_min;
        }
        else
        {
            y_min = poly2_y_min;
        }
    }
    else
    {
        //Then it has to be Vertical
        //y-coordinate is fixed and evaluate
        auto y1 = point_1.get<1>();
        auto y2 = point_2.get<1>();
        if (y1 < y2)
        {
            y_min = y1;
            y_max = y2;
        }
        else
        {
            y_min = y2;
            y_max = y1;
        }

        auto poly1_x_max = Geometry::xMax(poly1);
        auto poly2_x_max = Geometry::xMax(poly2);
        if (poly1_x_max > poly2_x_max)
        {
            x_max = poly1_x_max;
        }
        else
        {
            x_max = poly2_x_max;
        }

        auto poly1_x_min = Geometry::xMin(poly1);
        auto poly2_x_min = Geometry::xMin(poly2);
        if (poly1_x_min < poly2_x_min)
        {
            x_min = poly1_x_min;
        }
        else
        {
            x_min = poly2_x_min;
        }
    }

    //Generate Overlapping Polygon;
    std::list<Point> list_points{};
    Point bottom_left_corner = Geometry::createPoint(x_min, y_min);
    Point upper_left_corner = Geometry::createPoint(x_min, y_max);
    Point upper_right_corner = Geometry::createPoint(x_max, y_max);
    Point bottom_right_corner = Geometry::createPoint(x_max, y_min);
    list_points.push_back(bottom_left_corner);
    list_points.push_back(upper_left_corner);
    list_points.push_back(upper_right_corner);
    list_points.push_back(bottom_right_corner);
    list_points.push_back(bottom_left_corner);
    Polygon overlapping_polygon = Geometry::createPolygon(list_points);

    //Overlapping polygon is bigger and contains Polygon 1, P1 is within OP
    if (boost::geometry::within(poly1, overlapping_polygon))
    {
        poly1 = std::move(overlapping_polygon);
    }
    else
    {
        //It is the other way arround
        poly2 = std::move(overlapping_polygon);
    }

    //    std::cout << boost::geometry::dsv(poly1) << std::endl;
    //    std::cout << boost::geometry::dsv(poly2) << std::endl;
    //    std::cout << boost::geometry::dsv(overlapping_polygon) << std::endl;
    //
    return;
}

Geometry::Line_g Geometry::createLine(Point &pt1, Point &pt2)
{
    Line_g line(Point_g(pt1.x() - 1000 * (pt2.x() - pt1.x()), pt1.y() - 1000 * (pt2.y() - pt1.y())),
                Point_g(pt2.x() + 1000 * (pt2.x() - pt1.x()), pt2.y() + 1000 * (pt2.y() - pt1.y())));
    return std::move(line);
}

Geometry::Line_g Geometry::createLine(Point &&pt1, Point &&pt2)
{
    Line_g line(Point_g(pt1.x() - 1000 * (pt2.x() - pt1.x()), pt1.y() - 1000 * (pt2.y() - pt1.y())),
                Point_g(pt2.x() + 1000 * (pt2.x() - pt1.x()), pt2.y() + 1000 * (pt2.y() - pt1.y())));
    return std::move(line);
}

bool Geometry::intersect(Line_g &line1, Line_g &line2)
{
    bool result = bg::intersects(line1, line2);
    return result;
}

Geometry::Point Geometry::intersection(Line_g &line1, Line_g &line2)
{
    std::vector<Point_g> intersection_point;
    bg::intersection(line1, line2, intersection_point);
    auto x = intersection_point.at(0).get<0>();
    auto y = intersection_point.at(0).get<1>();
    return std::move(Geometry::createPoint(x, y));
}

//Computes the center of the line segment that is the intersection
//of two adjoint polygons
Geometry::Point Geometry::polygonDoorCenter(Polygon &P1, Polygon &P2)
{
    std::vector<Point_g> line_segment = Geometry::intersectingLineSegment(P1, P2);
    Point line_segment_center{};
    if (line_segment.size() == 0)
    {
        std::cout << "ERROR: Polygons do not intersect!" << std::endl;
        return line_segment_center;
    }
    auto x_0 = line_segment.at(0).get<0>();
    auto y_0 = line_segment.at(0).get<1>();
    auto x_1 = line_segment.at(1).get<0>();
    auto y_1 = line_segment.at(1).get<1>();

    return Geometry::createPoint((x_0 + x_1) * 0.5, (y_0 + y_1) * 0.5);
}

//Computes the line segment that is the intersection
//of two adjoint polygons
std::vector<Geometry::Point_g> Geometry::polygonDoorLineSegment(Polygon &P1, Polygon &P2)
{
    std::vector<Point_g> line_segment = Geometry::intersectingLineSegment(P1, P2);
    if (line_segment.size() == 0)
    {
        std::cout << "ERROR: Polygons do not intersect!" << std::endl;
        return line_segment;
    }

    return line_segment;
}

double Geometry::computeMeaningfulOrientation(bool aligned, const Point &landmark, Polygon &&P_middle, Polygon &&P_out)
{
    auto poli_exit_point = polygonDoorCenter(P_middle, P_out);

    auto yaw = angle(landmark, poli_exit_point);
    if (aligned)
    {
        return (yaw < 0) ? yaw + 2 * boost::math::constants::pi<double>() : yaw;
    }

    yaw = yaw + boost::math::constants::pi<double>();
    return (yaw < 0) ? yaw + 2 * boost::math::constants::pi<double>() : yaw;
}

Geometry::Point Geometry::computeMeaningfulLandmark(Polygon &&P_in, Polygon &&P_middle, Polygon &&P_out)
{
    auto entry_line_segment = Geometry::polygonDoorLineSegment(P_in, P_middle);
    auto exit_line_segment = Geometry::polygonDoorLineSegment(P_middle, P_out);

    auto perpendicular_line_to_entry_line_segment = Geometry::perpendicularLine(entry_line_segment);
    auto perpendicular_line_to_exit_line_segment = Geometry::perpendicularLine(exit_line_segment);

    auto p_middle_center = center(P_middle);

    auto p_middle_vertical_line = createLine(createPoint(p_middle_center.x(), p_middle_center.y()),
                                             createPoint(p_middle_center.x(), p_middle_center.y() + 1000));
    auto p_middle_horizontal_line = createLine(createPoint(p_middle_center.x(), p_middle_center.y()),
                                               createPoint(p_middle_center.x() + 1000, p_middle_center.y()));

    //Entry linesgment perpendicluar line intersection point to either vertical or horizontal
    //line that splits middle polygon in polygon in half
    Point entry_polygon_point{};
    if (Geometry::isLineSegmentHorizontal(entry_line_segment))
    { //Implies the perpendicular to line segment can interset the horizontal line of the polygon
        entry_polygon_point = intersection(perpendicular_line_to_entry_line_segment, p_middle_horizontal_line);
    }
    else
    {
        entry_polygon_point = intersection(perpendicular_line_to_entry_line_segment, p_middle_vertical_line);
    }

    //Exit linesgment perpendicluar line intersection point to either vertical or horizontal
    //line that splits middle polygon in polygon in half
    Point exit_polygon_point{};
    if (Geometry::isLineSegmentHorizontal(exit_line_segment))
    { //Implies the perpendicular to line segment can interset the horizontal line of the polygon
        exit_polygon_point = intersection(perpendicular_line_to_exit_line_segment, p_middle_horizontal_line);
    }
    else
    {
        exit_polygon_point = intersection(perpendicular_line_to_exit_line_segment, p_middle_vertical_line);
    }

    return Geometry::lineSegmentMiddle(entry_polygon_point, exit_polygon_point);
}

Geometry::Point Geometry::lineSegmentMiddle(Point pt1, Point pt2)
{
    return Geometry::createPoint((pt1.x() + pt2.x()) * 0.5, (pt1.y() + pt2.y()) * 0.5);
}

Geometry::Polygon *Geometry::expandAdjacentPolygonsMaximalOverlapStrangeCase(Polygon &poly1, Polygon &poly2)
{
    Polygon *p_overlap = nullptr;
    //Compute Intersecting Line Segment
    //const auto &line_seg = Geometry::intersectingLineSegment(poly1, poly2);
    auto line_seg = Geometry::intersectingLineSegment(poly1, poly2);
    for(auto &point : line_seg){ //Deal with rounding errors
        point.set<0>(round( point.get<0>()));
        point.set<1>(round( point.get<1>()) );
    }
    if (line_seg.size() == 0)
        return p_overlap;
    const Point_g &point_1 = line_seg.at(0);
    const Point_g &point_2 = line_seg.at(1);

    int x_min, x_max, y_min, y_max;

    if (Geometry::isLineSegmentHorizontal(line_seg))
    {
        //x-coordinate is fixed and evaluate
        auto x1 = point_1.get<0>();
        auto x2 = point_2.get<0>();
        if (x1 < x2)
        {
            x_min = x1;
            x_max = x2;
        }
        else
        {
            x_min = x2;
            x_max = x1;
        }

        auto poly1_y_max = Geometry::yMax(poly1);
        auto poly2_y_max = Geometry::yMax(poly2);
        if (poly1_y_max > poly2_y_max)
        {
            y_max = poly1_y_max;
        }
        else
        {
            y_max = poly2_y_max;
        }

        auto poly1_y_min = Geometry::yMin(poly1);
        auto poly2_y_min = Geometry::yMin(poly2);
        if (poly1_y_min < poly2_y_min)
        {
            y_min = poly1_y_min;
        }
        else
        {
            y_min = poly2_y_min;
        }
    }
    else
    {
        //Then it has to be Vertical
        //y-coordinate is fixed and evaluate
        auto y1 = point_1.get<1>();
        auto y2 = point_2.get<1>();
        if (y1 < y2)
        {
            y_min = y1;
            y_max = y2;
        }
        else
        {
            y_min = y2;
            y_max = y1;
        }

        auto poly1_x_max = Geometry::xMax(poly1);
        auto poly2_x_max = Geometry::xMax(poly2);
        if (poly1_x_max > poly2_x_max)
        {
            x_max = poly1_x_max;
        }
        else
        {
            x_max = poly2_x_max;
        }

        auto poly1_x_min = Geometry::xMin(poly1);
        auto poly2_x_min = Geometry::xMin(poly2);
        if (poly1_x_min < poly2_x_min)
        {
            x_min = poly1_x_min;
        }
        else
        {
            x_min = poly2_x_min;
        }
    }

    //Generate Overlapping Polygon;
    std::list<Point> list_points{};
    Point bottom_left_corner = Geometry::createPoint(x_min, y_min);
    Point upper_left_corner = Geometry::createPoint(x_min, y_max);
    Point upper_right_corner = Geometry::createPoint(x_max, y_max);
    Point bottom_right_corner = Geometry::createPoint(x_max, y_min);
    list_points.push_back(bottom_left_corner);
    list_points.push_back(upper_left_corner);
    list_points.push_back(upper_right_corner);
    list_points.push_back(bottom_right_corner);
    list_points.push_back(bottom_left_corner);
    Polygon overlapping_polygon = Geometry::createPolygon(list_points);

    //Check for strange case by computing union of polygons
    //starnge case occurs when extending either polygon we go to infeasibility
    auto original_union = polygonsUnion(poly1, poly2);
    auto extendP1_union = polygonsUnion(overlapping_polygon, poly2);
    auto extendP2_union = polygonsUnion(poly1, overlapping_polygon);

    if( (area(original_union) == area(extendP1_union)) ||
        (area(original_union) == area(extendP2_union)) )
    {
        //Overlapping polygon is bigger and contains Polygon 1, P1 is within OP
        if (boost::geometry::within(poly1, overlapping_polygon))
        {
            poly1 = std::move(overlapping_polygon);
        }
        else
        {
            //It is the other way arround
            poly2 = std::move(overlapping_polygon);
        }
        
    } else{//starnge case
        p_overlap = new Polygon();
        *p_overlap = overlapping_polygon;
    }
//    std::cout << boost::geometry::dsv(poly1) << std::endl;
//    std::cout << boost::geometry::dsv(poly2) << std::endl;
//    std::cout << boost::geometry::dsv(overlapping_polygon) << std::endl;

    // TODO: remove me , test start
    auto polygon = *p_overlap;
    double mm_to_m = 0.001;
    struct Line{ // y_var*y - m*x = b
      double y_var;
      double m; //slope
      double b; //y-axis coordinate
      int sign;
    };
    for(auto point_pt = polygon.begin(); point_pt != polygon.end()-1; ++point_pt) {
        auto point1 = *point_pt;
        auto point2 = *(point_pt + 1);
        Line l;
        if (point1.x() == point2.x()) {// vertical line (exception
            l.y_var = 0;
            l.m = 1;
            l.b = point1.x() * mm_to_m;
        } else { // normal case
            l.y_var = 1;
            l.m =
                -((double) point2.y() * mm_to_m - point1.y() * mm_to_m) / (point2.x() * mm_to_m - point1.x() * mm_to_m);
            l.b = point1.y() * mm_to_m + l.m * point1.x() * mm_to_m;
        }
        // Find the half-space
        // Find 3rd point
        Point point3;
        if ((point_pt + 1) == polygon.end() - 1) { // last point
            point3 = *(polygon.begin() + 2); // recall last point is equal to first point
        } else {
            point3 = *(point_pt + 2);
        }
        // Compute correct sign for half-space
        if (l.y_var * point3.y() * mm_to_m + l.m * point3.x() * mm_to_m - l.b < 0) {
            l.sign = 1;
        } else {
            l.sign = -1;
        }

        if (l.y_var * point3.y() * mm_to_m + l.m * point3.x() * mm_to_m - l.b == 0) {
            std::cerr << "[Interface MROPT: polygons are wrongly constructed, 3 collinear points" << std::endl;

//            auto *scfree_ = static_cast<mrrm::cfree::SimpleCfree*> (this->cfree_);//TODO: remove me
//            PolygonSet pset {polygon};
//            scfree_->plotRectangles(pset );//TODO: remove mer
        }
    }
    auto ar = area(*p_overlap);
    if(area(*p_overlap) < 1000000){//TODO: remove me
      int a = 1;
    }
    return p_overlap;
}
