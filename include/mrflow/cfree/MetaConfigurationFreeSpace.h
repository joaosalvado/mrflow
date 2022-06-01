/*
 * File:   MetaConfigurationFreeSpace.h
 * Author: Joao Salvado
 *
 * This class intends to generate a new configuration free space where the original
 * intersecting poligons will be transformed into adjoint polygons.
 * E.g. P1 and P2 are original poligons and P1 intersect P2 = P3 then
 * I = {P3}
 * S = {P1\P3, P2\P2}
 * 
 * Created on April 1, 2019, 10:58 AM
 */
#pragma once
#ifndef METACONFIGURATIONFREESPACE_H
#define METACONFIGURATIONFREESPACE_H

#include <unordered_map>
#include "ConfigurationFreeSpace.h"
#include <boost/functional/hash.hpp>

namespace mrflow
{
    namespace cfree
    {

        class MetaConfigurationFreeSpace : public ConfigurationFreeSpace
        {
        public:
            //The original polygons are read from a file where a sequence of xy points
            //are stated clockwise.
            MetaConfigurationFreeSpace(const std::string filename) : ConfigurationFreeSpace(filename){};
            MetaConfigurationFreeSpace() : ConfigurationFreeSpace(){};
            virtual ~MetaConfigurationFreeSpace(){};

            /**
             * Polygons provided in the given file are already adjacent
             */
            void createMetaConnectivityMap_OverlapingPolygons();

            /**
             * Polygons provided in the given file are already adjacent
             */
            void createMetaConnectivityMap_AdjacentPolygons();

            /**
             * Test if polygon p_id is an intersection polygon (in set I)
             * @param p_id
             * @return
             */
            bool isIntersection(int p_id)
            {
                auto it = std::find(this->_Itersections_Ids.begin(), this->_Itersections_Ids.end(), p_id);
                if (it != this->_Itersections_Ids.end())
                {
                    return true;
                }
                return false;
            }

            /**
             * Get meta polygon, this is a polygon that might not be the original
             * belongs to the new set of polygons that was generated based on some principle
             * @param p_id
             * @return
             */
            Polygon getMetaPolygon(int p_id)
            {
                return this->_metaPolygons.at(p_id);
            }

            /**
             * Get the set of this newly created polygons
             * @return
             */
            PolygonSet &getMetaPolygonSet()
            {
                return this->_metaPolygons;
            }

            /**
             * Get a vector of polygon ids representing the polygons adjacent to polygon p_id
             * @param p_id
             * @return
             */
            std::vector<int> getAdjointPolygons(int p_id) const
            {
                return this->_metaConnectivityMap.at(p_id);
            }

            /**
             * Get the amount of polygons
             * @return
             */
            std::size_t getNumberOfMetaPolygons(void) const
            {
                return this->_numberMetaPolygons;
            }

            /**
             * Returns the meta polygon id where point (x,y) is located
             * Otherwise returns -1
             * @param x
             * @param y
             * @return
             */
            int withinMetaPolygon(int x, int y)
            {
                for (int p_id = 0; p_id < _metaPolygons.size(); ++p_id)
                {
                    if (boost::polygon::contains(this->_metaPolygons.at(p_id),
                                                 boost::polygon::construct<Point>(x, y)))
                        return p_id;
                }
                return -1;
            }

            //Test is two polygons are ajoint/connected
            /**
             * Checks if two polygons are adjacent/connected
             * @param polyId_1
             * @param polyId_2
             * @return
             */
            bool areMetaPolygonsConnected(int polyId_1, int polyId_2)
            {
                for (auto it = this->_metaConnectivityMap.at(polyId_1).begin();
                     it != this->_metaConnectivityMap.at(polyId_1).end(); ++it)
                {
                    if (polyId_2 == *it)
                    {
                        return true;
                    }
                }
                return false;
            }

            void eraseConnection(int polyId_1, int polyId_2)
            {
                auto it = std::find(
                    this->_metaConnectivityMap.at(polyId_1).begin(),
                    this->_metaConnectivityMap.at(polyId_1).end(),
                    polyId_2);
                if (it != this->_metaConnectivityMap.at(polyId_1).end())
                {
                    this->_metaConnectivityMap.at(polyId_1).erase(it);
                }

                it = std::find(
                    this->_metaConnectivityMap.at(polyId_2).begin(),
                    this->_metaConnectivityMap.at(polyId_2).end(),
                    polyId_1);
                if (it != this->_metaConnectivityMap.at(polyId_2).end())
                {
                    this->_metaConnectivityMap.at(polyId_2).erase(it);
                }
            }

            /**
             * Check if state angle theta is on the direction P_1 to P_2 -pi/2 to pi/2
             * @param state
             * @param P_1
             * @param P_2
             * @return
             */
            //bool stateAligned(const ompl::base::State *state, Polygon &&P_1, Polygon &&P_2);

            /**
             * Note: door is to be considered the line segment of two intersecting polygons.
             * In this function all door centers of intersecting meta polygons are computed
             * and save in an unorered map;
             */
            void computeAllDoorCenters();
            void computeAllDoorCenters_overlappingPolygons();

            /**
            * Get the center of two intersecting polygons that are in a precmputed hash map
            * @param p1
            * @param p2
            * @return
            */
            Point getCenterDoor(int p1, int p2) const
            {
                std::pair<int, int> key = std::make_pair(p1, p2);
                return this->m_door_center.at(key);
            }

            /**
             * @brief Get the Cfree ma x-coordinate
             * 
             * @return int 
             */
            int getCfreeMaxX() const;

            /**
             * @brief Get the Cfree min x-coordinate
             * 
             * @return int 
             */
            int getCfreeMinX() const;

            /**
             * @brief Get the Cfree Max y-coordinate
             * 
             * @return int 
             */
            int getCfreeMaxY() const;

            /**
             * @brief Get the Cfree Min Y-coordiante
             * 
             * @return int 
             */
            int getCfreeMinY() const;

            /**
             * @brief Prints polygon points, 1 line per polygon
             * 
             */
            void printMetaPolygons()
            {
                for (const auto &mp : this->_metaPolygons)
                {
                    std::cout << boost::geometry::dsv(mp) << std::endl;
                }
            }

            std::vector<std::vector<int>> connectivity_graph(PolygonSet polygons);
            std::vector<std::vector<int>> connectivity_graph_general(
                    std::vector<std::shared_ptr<Polygon>> polygons);
        protected:
            //Set of meta Polygons
            PolygonSet _metaPolygonsOriginal;
            //Intersections;
            std::vector<int> _Itersections_Ids;
            //Singletons;
            std::vector<int> _Singletons_Ids;
            //Connectivity graph of the new created polygons
            PolygonConnectivityGraph _metaConnectivityMapOriginal;
            //Connectivity algorithm
            gtl::connectivity_extraction_90<int> _metaPolygonConnectivityAlgorithmOriginal;
            //Number of Polygons
            std::size_t _numberMetaPolygonsOriginal;

            //IMPROVED Polygons

            //Set of meta Polygons
            PolygonSet _metaPolygons;
            //Composition of Improved Polygons
            std::vector<std::list<int>> _compositionTable;
            //Connectivity graph of the new created polygons
            PolygonConnectivityGraph _metaConnectivityMap;
            //Connectivity algorithm
            gtl::connectivity_extraction_90<int> _metaPolygonConnectivityAlgorithm;
            //Number of Polygons
            std::size_t _numberMetaPolygons;


            /**
             * Polygons provided in the file are overlapping
             */
            void createMetaConnectivityMap();

            /**
             * Transform overlapping into adjacent polygons
             */
            void generateImprovedCfreeMap();

            /**
             * Align the Points of the meta polygons such that the
            *  left corner comes first.
             */
            void leftCornerFirstAlignement();

            /**
             * @brief Test if two polygons can be traversersed by robot
             * 
             * @param pol1 
             * @param pol2 
             * @return true 
             * @return false 
             */
            bool arePolygonsConnected(const Polygon &pol1, const Polygon &pol2);

            void modifyConnectivityConnected();

            //Hash table of the center of the intersecting line segment of polygons p1 and p2
            struct HashPolygonPair
            {
                std::size_t operator()(const std::pair<int, int> &k) const
                {
                    using boost::hash_combine;
                    using boost::hash_value;

                    // Start with a hash value of 0    .
                    std::size_t seed = 0;
                    auto &&min = std::min({k.first, k.second});
                    auto &&max = std::max({k.first, k.second});
                    // Modify 'seed' by XORing and bit-shifting in
                    // one member of 'Key' after the other:
                    // min and max used for ordering key, which entails order doesnt matter
                    // center of P1 Intersect P2 is the same  center of P2 Intersect P1
                    hash_combine(seed, hash_value(min));
                    hash_combine(seed, hash_value(max));
                    // Return the result.
                    return seed;
                }
            };
            struct EqualToPair
            {
                bool operator()(const std::pair<int, int> &k1, const std::pair<int, int> &k2) const
                {
                    auto &&min1 = std::min({k1.first, k1.second});
                    auto &&max1 = std::max({k1.first, k1.second});
                    auto &&min2 = std::min({k2.first, k2.second});
                    auto &&max2 = std::max({k2.first, k2.second});
                    if (min1 == min2 && max1 == max2)
                        return true;
                    return false;
                }
            };

            //Unordered Map/ Hash map of all the door centers of the topological graph of the meta polygons
            std::unordered_map<std::pair<int, int>, Point, HashPolygonPair, EqualToPair> m_door_center;
        };
    } // namespace cfree
} // namespace mrrm

#endif /* METACONFIGURATIONFREESPACE_H */
