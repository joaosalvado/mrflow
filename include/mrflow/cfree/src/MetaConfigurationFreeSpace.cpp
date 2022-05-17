/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MetaConfigurationFreeSpace.cpp
 * Author: ohmy
 * 
 * Created on April 1, 2019, 10:58 AM
 */

#include <vector>
#include <math.h>

#include "mrflow/cfree/MetaConfigurationFreeSpace.h"
#include <fstream>

//Generating meta polygons state, voronoi alike discretization
using namespace mrflow::cfree;

void MetaConfigurationFreeSpace::createMetaConnectivityMap()
{
    //Set of Polygons which are a result of an intersection, set I
    PolygonSet _Intersections;
    //Set of Polygons which are a result of subtracting the intersections on the original polygons, set S
    PolygonSet _SimplePoligons;

    //Test if original polygons are empty
    if (_polygonSet.empty())
    {
        std::cout << "The Original configuration free space doesn't have polygons" << std::endl;
        return;
    }

    //Copy original Polygons
    _SimplePoligons = this->_polygonSet;

    //Computing all Intersections
    PolygonSet allIntersections{};
    for (int p_id = 0; p_id < this->_connectivityMap.size(); p_id++)
    {

        for (int p_id_neighbor : _connectivityMap.at(p_id))
        {
            Polygon Intersection = this->polygonsIntersection(this->getPolygon(p_id), this->getPolygon(p_id_neighbor));
            allIntersections.push_back(Intersection);
        }
    }

    for (auto &p1 : allIntersections)
    {
        if (_Intersections.empty())
        {
            _Intersections.push_back(p1);
            continue;
        }

        bool addPolygon = true;
        for (auto &p2 : _Intersections)
        {
            PolygonSet ps1{p1}, ps2{p2};
            if (boost::polygon::equivalence(ps1, ps2))
            {
                addPolygon = false;
                break;
            }
        }

        if (addPolygon)
        {
            _Intersections.push_back(p1);
        }
    }

    //
    PolygonSet intersectionsAndOriginalPolygons{};
    intersectionsAndOriginalPolygons = this->_polygonSet;
    for (auto &p : _Intersections)
    {
        PolygonSet ps{p};
        //ps = boost::polygon::shrink(ps, 500);
        intersectionsAndOriginalPolygons.push_back(ps.front());
    }

    //Insert Polygons for connectivity algorithm
    for (Polygon poly : intersectionsAndOriginalPolygons)
    {
        _metaPolygonConnectivityAlgorithmOriginal.insert(gtl::view_as<gtl::polygon_90_concept>(poly));
    }

    std::vector<std::set<int>> graph(intersectionsAndOriginalPolygons.size());

    //populate the graph with edge data -> less expensive computation, more difficult to work with
    _metaPolygonConnectivityAlgorithmOriginal.extract(graph);

    this->_metaConnectivityMapOriginal.resize(intersectionsAndOriginalPolygons.size());
    //Change the structure of the graph to a vector of vectors
    int polyId = 0;
    for (std::set<int> adjacencyList : graph)
    {
        for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
        {
            this->_metaConnectivityMapOriginal[polyId].push_back(*it);
        }
        ++polyId;
    }

    //Delete all connections to simple Polygons
    for (int p_id = 0; p_id < _SimplePoligons.size(); p_id++)
    {
        this->_metaConnectivityMapOriginal.at(p_id).clear();
    }

    //Change connectivity graph and recompute simple polygons
    for (int pI_id = 0; pI_id < _Intersections.size(); ++pI_id)
    {
        auto neighborPolygons = this->_metaConnectivityMapOriginal.at(this->_metaConnectivityMapOriginal.size() - _Intersections.size() + pI_id);
        for (int pS_id : neighborPolygons)
        {
            Polygon pS = _SimplePoligons.at(pS_id);
            Polygon pI = _Intersections.at(pI_id);
            PolygonSet psS_new, psS{pS}, psI{pI};
            assign(psS_new, psS - pI);

            _SimplePoligons.at(pS_id) = psS_new.front();
            _metaConnectivityMapOriginal.at(pS_id).push_back(this->_metaConnectivityMapOriginal.size() - _Intersections.size() + pI_id);
        }
    }

    //Generate goal polygon meta set and set of ids for intersections and singletons
    this->_numberMetaPolygonsOriginal = _SimplePoligons.size() + _Intersections.size();

    int p_id = 0;
    for (int p_s_id = 0; p_s_id < _SimplePoligons.size(); ++p_s_id)
    {
        this->_Singletons_Ids.push_back(p_id++);
        this->_metaPolygonsOriginal.push_back(_SimplePoligons.at(p_s_id));
    }
    for (int p_s_id = 0; p_s_id < _Intersections.size(); ++p_s_id)
    {
        this->_Itersections_Ids.push_back(p_id++);
        this->_metaPolygonsOriginal.push_back(_Intersections.at(p_s_id));
    }
}

void MetaConfigurationFreeSpace::generateImprovedCfreeMap()
{

    //    auto P = this->convertPolygonGeometric(this->_metaPolygonsOriginal.at(0));
    //    auto Pg = this->convertGeometricPolygon(P);
    //    this->isConvex(this->_metaPolygonsOriginal.at(0));

    //Reverse Original Polygons - Clockwise
    for (int po_id = 0; po_id < this->_metaPolygonsOriginal.size(); po_id++)
    {
        Polygon &polygon_original = this->_metaPolygonsOriginal.at(po_id);
        std::list<Point> l_pts;
        for (auto point : boost::adaptors::reverse(polygon_original.coords_))
        {
            l_pts.push_back(point);
        }
        polygon_original = std::move(this->createPolygon(l_pts));
    }

    for (int po_id = 0; po_id < this->_metaPolygonsOriginal.size(); po_id++)
    {
        Polygon polygon_original = this->_metaPolygonsOriginal.at(po_id);
        if (this->_metaPolygons.empty())
        {
            this->_metaPolygons.push_back(polygon_original);
            std::list<int> l({po_id});
            this->_compositionTable.push_back(std::move(l));
            continue;
        }

        bool added_to_existing_meta_polygon = false;
        for (int mp_id = 0; mp_id < this->_metaPolygons.size(); ++mp_id)
        {
            Polygon &meta_polygon = this->_metaPolygons.at(mp_id);
            Polygon union_poly;
            bool convex = this->unionConvex(meta_polygon, polygon_original, union_poly);
            if (convex)
            {
                meta_polygon = std::move(union_poly);
                this->_compositionTable.at(mp_id).push_back(po_id);
                added_to_existing_meta_polygon = true;
                break;
            }
        }

        if (!added_to_existing_meta_polygon)
        {
            this->_metaPolygons.push_back(polygon_original);
            std::list<int> l({po_id});
            this->_compositionTable.push_back(std::move(l));
        }
    }

    std::vector<int> deleted{};
    for (int mp_id_1 = 0; mp_id_1 < this->_metaPolygons.size(); ++mp_id_1)
    {
        auto result = std::find(deleted.begin(), deleted.end(), mp_id_1);
        if (result != deleted.end())
            continue;
        Polygon &meta_polygon_1 = this->_metaPolygons.at(mp_id_1);
        for (int mp_id_2 = mp_id_1 + 1; mp_id_2 < this->_metaPolygons.size(); ++mp_id_2)
        {
            Polygon &meta_polygon_2 = this->_metaPolygons.at(mp_id_2);
            Polygon union_poly;
            bool convex = this->unionConvex(meta_polygon_1, meta_polygon_2, union_poly);
            if (convex)
            {
                meta_polygon_1 = std::move(union_poly);
                for (auto po_id : this->_compositionTable.at(mp_id_2))
                {
                    this->_compositionTable.at(mp_id_1).push_back(po_id);
                }
                deleted.push_back(mp_id_2);
                break;
            }
        }
    }

    for (int i = 0; i < deleted.size(); ++i)
    {
        int po_to_delete = deleted.at(i);
        this->_metaPolygons.erase(this->_metaPolygons.begin() + po_to_delete - i);
        this->_compositionTable.erase(this->_compositionTable.begin() + po_to_delete - i);
    }

    this->_numberMetaPolygons = this->_metaPolygons.size();

    this->createMetaConnectivityMap_ExpandedPolygons();
    this->leftCornerFirstAlignement();

    //create map
    this->computeAllDoorCenters();

    // for (const auto &mp : this->_metaPolygons) {
    //     std::cout << boost::geometry::dsv(mp) << std::endl;//TODO: comment me
    // }
}

void MetaConfigurationFreeSpace::createMetaConnectivityMap_ExpandedPolygons()
{
    //Insert Polygons for connectivity algorithm

    for (Polygon &poly : this->_metaPolygons)
    {
        //gtl::scale_up(poly, 1.01);
        _metaPolygonConnectivityAlgorithm.insert(gtl::view_as<gtl::polygon_90_concept>(poly));
    }

    std::vector<std::set<int>> graph(this->_metaPolygons.size());

    //populate the graph with edge data -> less expensive computation, more difficult to work with
    _metaPolygonConnectivityAlgorithm.extract(graph);

    this->_metaConnectivityMap.resize(this->_metaPolygons.size());
    //Change the structure of the graph to a vector of vectors
    int polyId = 0;
    for (std::set<int> adjacencyList : graph)
    {
        for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it)
        {
            this->_metaConnectivityMap[polyId].push_back(*it);
        }
        ++polyId;
    }
}

void MetaConfigurationFreeSpace::createMetaConnectivityMap_OverlapingPolygons()
{
    this->createMetaConnectivityMap();
    this->generateImprovedCfreeMap();
}

void MetaConfigurationFreeSpace::createMetaConnectivityMap_AdjacentPolygons()
{
    this->_metaPolygons = this->_polygonSet;
    this->createMetaConnectivityMap_ExpandedPolygons();

    this->_numberMetaPolygons = this->_metaPolygons.size();

    this->leftCornerFirstAlignement();

    this->modifyConnectivityConnected(); //TODO check this when reading from file

    //create map
    this->computeAllDoorCenters();
}

//Align the Points of the meta polygons such that the
//left corner comes first.
void MetaConfigurationFreeSpace::leftCornerFirstAlignement()
{
    for (auto &mp_p : this->_metaPolygons)
    {
        //Find left corner position
        int left_corner_pos{0}, it_point{0};
        int min_corner{1000000000};

        for (const auto &mp_point : mp_p.coords_)
        {
            double x = mp_point.x();
            double y = mp_point.y();
            auto temp_min = std::sqrt(x * x + y * y);
            if (temp_min < min_corner)
            {
                min_corner = temp_min;
                left_corner_pos = it_point;
            }
            it_point++;
        }

        auto mp = this->convertPolygonGeometric(mp_p);

        auto last_point = boost::geometry::points_end(mp);
        --last_point;
        auto begin_point = boost::geometry::points_begin(mp);

        //std::cout << boost::geometry::dsv(mp) << std::endl;
        while (left_corner_pos != 0)
        {

            for (auto point_it = begin_point; point_it != boost::geometry::points_end(mp); ++point_it)
            {
                if (point_it == begin_point)
                {
                    continue;
                }
                //Swap
                auto prev_point = --point_it;
                ++point_it;
                auto next_point = point_it;

                bg::set<0>(*prev_point, bg::get<0>(*next_point));
                bg::set<1>(*prev_point, bg::get<1>(*next_point));
            }
            bg::set<0>(*last_point, bg::get<0>(*begin_point));
            bg::set<1>(*last_point, bg::get<1>(*begin_point));
            left_corner_pos--;
        }

        auto mp_p_new = this->convertGeometricPolygon(mp);
        mp_p = std::move(mp_p_new);
    }
}

void MetaConfigurationFreeSpace::computeAllDoorCenters()
{
    int P = this->getNumberOfMetaPolygons();
    if (P == 0)
    {
        std::cerr << "There are no meta polygons" << std::endl;
        return;
    }
    for (int p1_id = 0; p1_id < P; ++p1_id)
    {
        for (int p2_id = p1_id + 1; p2_id < P; ++p2_id)
        {
            if (this->areMetaPolygonsConnected(p1_id, p2_id))
            {
                if (this->m_door_center.find({p1_id, p2_id}) == this->m_door_center.end())
                { //If the key doesn't exist
                    auto &&P1 = this->getMetaPolygon(p1_id);
                    auto &&P2 = this->getMetaPolygon(p2_id);
                    std::vector<Point_g> line_segment = this->intersectingLineSegment(P1, P2);
                    if (line_segment.size() == 0)
                    { //No intersection
                        continue;
                    }
                    else
                    {
                        auto x_0 = line_segment.at(0).get<0>();
                        auto y_0 = line_segment.at(0).get<1>();
                        auto x_1 = line_segment.at(1).get<0>();
                        auto y_1 = line_segment.at(1).get<1>();

                        auto &&center = this->createPoint((x_0 + x_1) * 0.5, (y_0 + y_1) * 0.5);
                        m_door_center[{p1_id, p2_id}] = std::move(center);
                    }
                }
            }
        }
    }
}

/*bool MetaConfigurationFreeSpace::stateAligned(const ompl::base::State *state, Polygon &&P_1, Polygon &&P_2)
{
    auto *rsState = static_cast<const mrrm::state::SingleRobotState::StateType *>(state);
    auto robot_point = this->createPoint(rsState->getX(), rsState->getY());
    auto poli_exit_point = polygonDoorCenter(P_1, P_2);

    auto angle_1 = rsState->getYaw();
    auto angle_2 = this->angle(robot_point, poli_exit_point);

    //Difference from 0 to 180
    double phi = std::abs(angle_1 - angle_2);
    if (phi >= 2 * boost::math::constants::pi<double>())
        phi -= 2 * boost::math::constants::pi<double>();
    double difference = phi > boost::math::constants::pi<double>() ? 2 * boost::math::constants::pi<double>() - phi : phi;
    if (difference > boost::math::constants::pi<double>() / 2)
        return false;
    else
    {
        return true;
    }
}*/

int MetaConfigurationFreeSpace::getCfreeMaxX() const
{
    int x_max = -1;
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id)
    {
        const auto &x_max_candidate = this->xMax(this->_metaPolygons[p_id]);
        if (x_max < x_max_candidate)
            x_max = x_max_candidate;
    }
    return x_max;
}

int MetaConfigurationFreeSpace::getCfreeMinX() const
{
    int x_min = -1;
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id)
    {
        const auto &x_min_candidate = this->xMin(this->_metaPolygons[p_id]);
        if (x_min == -1)
            x_min = x_min_candidate;
        if (x_min > x_min_candidate)
            x_min = x_min_candidate;
    }
    return x_min;
}
int MetaConfigurationFreeSpace::getCfreeMaxY() const
{
    int y_max = -1;
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id)
    {
        const auto &y_max_candidate = this->yMax(this->_metaPolygons[p_id]);
        if (y_max < y_max_candidate)
            y_max = y_max_candidate;
    }
    return y_max;
}
int MetaConfigurationFreeSpace::getCfreeMinY() const
{
    int y_min = -1;
    for (int p_id = 0; p_id < this->getNumberOfMetaPolygons(); ++p_id)
    {
        const auto &y_min_candidate = this->yMin(this->_metaPolygons[p_id]);
        if (y_min == -1)
            y_min = y_min_candidate;
        if (y_min > y_min_candidate)
            y_min = y_min_candidate;
    }
    return y_min;
}

bool MetaConfigurationFreeSpace::arePolygonsConnected(const Polygon &pol1, const Polygon &pol2)
{
    PolygonSet p1{pol1}, p2{pol2};
    PolygonSet ps3;
    assign(ps3, p1 & p2);
    //Since polygons will not have inner rings / holes and are convex then the intersection is always ONE polygon

    auto line = this->intersectingLineSegment(pol1, pol2);
    if (line.size() == 0) //No intersection at all
    {
        std::cout << 0;
        return false;
    }

    if (line.size() == 1) //Point
    {
        std::cout << 1;
        return false;
    }
    if (line.size() == 2) //Line
    {
        auto x1 = line.at(0).get<0>();
        auto y1 = line.at(0).get<1>();
        auto x2 = line.at(1).get<0>();
        auto y2 = line.at(1).get<1>();
        auto distance =
            std::sqrt(
                (x2 - x1) * (x2 - x1) +
                (y2 - y1) * (y2 - y1));
        std::cout << distance;
        if (distance > 1.1 * this->footprint_->getLength())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    std::cerr << "[MetaCfree] More than 2 points, bad polygons" << std::endl;
    auto area_ps3 = gtl::area(ps3.front());
    return true;
}

void MetaConfigurationFreeSpace::modifyConnectivityConnected()
{
    for (int p1 = 0; p1 < this->getNumberOfMetaPolygons(); ++p1)
    {
        for (int p2 = 0; p2 < this->getNumberOfMetaPolygons(); ++p2)
        {
            if (this->areMetaPolygonsConnected(p1, p2))
            {
                std::cout << p1 << " " << p2 << " - ";
                if (!arePolygonsConnected(getMetaPolygon(p1), getMetaPolygon(p2)))
                {
                    this->eraseConnection(p1, p2);
                }
                std::cout << std::endl;
            }
        }
    }
}
