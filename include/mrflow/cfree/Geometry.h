//
// Created by ohmy on 2020-10-27.
//

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <boost/polygon/polygon.hpp>
#include <boost/assign.hpp>
#include <math.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace bg = boost::geometry;
namespace gtl = boost::polygon;
using namespace boost::polygon::operators;

namespace mrflow {
namespace cfree{

//Bounding box bounds
struct Bounds {
    double xMin;
    double xMax;
    double yMin;
    double yMax;
};

class Geometry
{
public:
    //Boost polygon geometric types
    typedef gtl::polygon_data<int> Polygon;
    typedef gtl::polygon_traits<Polygon>::point_type Point;
    typedef std::vector<Polygon> PolygonSet;
    typedef std::vector<std::vector<int> > PolygonConnectivityGraph;

    //Boost geometric models
    typedef bg::model::point<double, 2, bg::cs::cartesian> Point_g;
    typedef bg::model::polygon<Point_g> Polygon_g;
    typedef bg::model::segment<Point_g> Line_g;


    Geometry() = default;
    virtual ~Geometry() = default;


    /**
     * Computes the intersection between two polygons
     * Assume polygons are intersecting over some region (e.g. not a line or point)
     * @param poly1
     * @param poly2
     * @return
     */
     static Polygon polygonsIntersection(const Polygon &poly1, const Polygon &poly2);

    /**
     * Union of two adjancent polygons poly1 and poly2 in a reduced form
     * where colinear points are not allowed
     * @param poly1
     * @param poly2
     * @return
     */
    static Polygon polygonsUnion(const Polygon &poly1, const Polygon &poly2);

    /**
     * Intersection of a set of polygons
     * @param Polygons
     * @return
     */
    static Polygon polygonSetIntersection(const PolygonSet &Polygons);


    static Polygon convexhull( Polygon polygon );

    static PolygonSet polygonMinus(
            Polygon convexhull,
            Polygon unionpol);

    static PolygonSet split( Polygon pol);


    /**
    * Translate polygon poly 4 points into x y bounds
    * @param poly
    * @return
    */
    static Bounds boundaries(const Polygon &poly);


    /*
     * Return x-coordinate of Point p
     */
    static int getX(const Point &p) {
        return gtl::x(p);
    }

    /*
     * Return y-coordinate of Point p
     */
    static int getY(const Point &p) {
        return gtl::y(p);
    }

    /**
     * Checks if two polygons are intersecting
     * @param poly1
     * @param poly2
     * @return
     */
    static bool areIntersecting(const Polygon &poly1, const Polygon &poly2);


    /**
     * Converts Boost polygon into a list of points
     *
     * @param poly
     * @return
     */
    static std::list<std::pair<int, int>> convertPolygon(Polygon poly);


    /**
     * Angle of a vector from point p1 to p2
     * @param p1
     * @param p2
     * @return
     */
    static double angle(const Point &p1, const Point &p2);


    /**
     * Size of the maximum size of a given Polygon
     * @param poly
     * @return
     */
    static int maximumSide(const Polygon &poly);


    /**
     * Get Center of a Polygons
     * @param poly
     * @return
     */
    static Point center(const Polygon &poly);

    /**
     * Get Area of a polygon
     * @param poly
     * @return
     */
    static double area(const Polygon &poly);


    /**
     * Get vector with 2 points which are the start and end of a line segment that is
     * the intersection of two adjacent polygons
     * @param P_1
     * @param P2
     * @return
     */
    static std::vector<Point_g> intersectingLineSegment(const Polygon &P_1, const Polygon &P2);

    /**
     * Checks is line segment is horiztonal y_0 = y_1
     * @param line_segment
     * @return
     */
    static bool isLineSegmentHorizontal(const std::vector<Point_g> &line_segment);


    /**
     * Expand adjacent polygon such that they overlap maximally and the union of both remains the same
     * @param poly1
     * @param poly2
     */
    static void expandAdjacentPolygonsMaximalOverlap(Polygon &poly1, Polygon &poly2);

    static Polygon* expandAdjacentPolygonsMaximalOverlapStrangeCase(Polygon &poly1, Polygon &poly2);

    /**
     * Get minimal x-coordinate of polygon poly
     * @param poly
     * @return
     */
    static int xMin(const Polygon &poly);

    /**
     * Get maximal x-coordinate of polygon poly
     * @param poly
     * @return
     */
    static int xMax(const Polygon &poly);

    /**
     * Get minimal y-coordinate of polygon poly
     * @param poly
     * @return
     */
    static int yMin(const Polygon &poly);

    /**
     * Get maximal y-coordinate of polygon poly
     * @param poly
     * @return
     */
    static int yMax(const Polygon &poly);


    /**
            * Creates boost point out of xy-coordinate
            * @param x
            * @param y
            * @return
            */
    static Point createPoint(int x, int y);

    /**
     * Creates a boost polygon out of a list of boost points
     * @param listPoints
     * @return
     */
    static Polygon createPolygon(std::list<Point> &listPoints);
    static Polygon createPolygon(std::list<Point> &&listPoints);

    /**
             * Convert polygon from Boost::Polygon to Boost::Geometry
             * @param poly
             * @return
             */
    static Polygon_g convertPolygonGeometric(const Polygon &poly);

    /**
     * Convert polygon from Boost::Geometry to Boost::Polygon
     * @param poly
     * @return
     */
    static Polygon convertGeometricPolygon(const Polygon_g &poly);

    /**
     * Checks if polygon is convex
     * @param poly
     * @return
     */
    static bool isConvex(const Polygon &poly);

    /**
     * Creates a Boost::Geometry Line out of Boost::Polygon points
     * @param pt1
     * @param pt2
     * @return
     */
    static Line_g createLine(Point &pt1, Point &pt2);
    static Line_g createLine(Point &&pt1, Point &&pt2);

    /**
     * Checks if 2 Boost::Geometry Lines intersect
     * @param line1
     * @param line2
     * @return
     */
    static bool intersect(Line_g &line1, Line_g &line2);

    /**
     * Computes the intersection point of two Boost::Geometry lines that are not parallel
     * @param line1
     * @param line2
     * @return
     */
    static Point intersection(Line_g &line1, Line_g &line2);

    //

    /**
     * Compute the union of p1 and p2 if such union is convex
     * Otherwise union_p1_p2 remains empty and returns false
     * @param p1
     * @param p2
     * @param union_p1_p2
     * @return
     */
    static bool unionConvex(const Polygon &p1, const Polygon &p2, Polygon &union_p1_p2);

    /**
     * Creates perpendicular line to line segment, assumes vertical or horizontal line segments
     * @param line_segment
     * @return
     */
    static Line_g perpendicularLine(std::vector<Point_g> &line_segment);



    /**
            * Having a robot transitioning polygons P_in --> P_middle --> P_out
            * One has to define a meaningful Point (x,y) in P_middle
            * Compute as follows, let us have three line segments(ls):
            * ls1 - center(P_in) and center(door(P_in, P_middle))
            * ls2 - center(P_out) and center(door(P_out, P_middle))
            * ls3_h - horizontal line cutting P_middle in half
            * ls3_v - vertical line cutting P_middle in half
            * p1 = (ls1 intersect ls3_h) or (ls1 intersect ls3_v)
            * p2 = (ls2 intersect ls3_h) or (ls2 intersect ls3_v)
            * ls_final - p1 to p2
            * return center(ls_final)
            * @param P_in
            * @param P_middle
            * @param P_out
            * @return
            */
    static Point computeMeaningfulLandmark(Polygon &&P_in, Polygon &&P_middle, Polygon &&P_out);

    /**
     * Returns the angle between landmark ("center of P_middle") and door(P_middle, P_out)
     * @param aligned
     * @param landmark
     * @param P_middle
     * @param P_out
     * @return
     */
    static double computeMeaningfulOrientation(bool aligned, const Point &landmark, Polygon &&P_middle, Polygon &&P_out);

    /**
     * Door refers to the line segment of two intersecting polygons
     * This returns the center
     * @param P1
     * @param P2
     * @return
     */
    static Point polygonDoorCenter(Polygon &P1, Polygon &P2);

    /**
     * Return door line segment as BOOST::Geometry points
     * @param P1
     * @param P2
     * @return
     */
    static std::vector<Point_g> polygonDoorLineSegment(Polygon &P1, Polygon &P2);




    /**
     * Middle point between two points
     * @param pt1
     * @param pt2
     * @return
     */
    static  Point lineSegmentMiddle(Point pt1, Point pt2);


};

}}


#endif //GEOMETRY_H
