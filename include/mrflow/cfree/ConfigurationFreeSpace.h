/*
 * File:   ConfigurationFreeSpace.h
 * Author: João Salvado
 * 
 * Configurartion Free Space is defined as a set of overlapping rectangular polygons.
 * Other utilities are provided in this class such as:
 *  Polygon operations: union, intersection, within, center
 *  Genate a topology graph based on the overlapping of the polygons.
 *  others: compute max number of robots in polygon
 *
 * Note: Polygon Points are int and defined in milimeters
 * Note: Polygons are read from a file
 * For the definition of polygons we utilized Boost Polygon framework
 * We also utilize Boost Geometry Library for other computations that are still not available in boost polygon
 * 
 * Created on August 17, 2018, 2:56 PM
 */
#pragma once
#ifndef CONFIGURATIONFREESPACE_H
#define CONFIGURATIONFREESPACE_H

#include "mrflow/cfree/Geometry.h"

//Forward Declaration
class SingleRobotState;
class VehicleFootprint;


class VehicleFootprint {
public:
    VehicleFootprint(double length, double width)
            : length_(length), width_(width){ };

    /**
     * Return circle radius
     * @return
     */
    double getCircleRadius() const { return this->radius_; };

    /**
     * Return length of footprint
     * @return
     */
    int getLength() const { return this->length_; }

    /**
     * Return width of footprint
     * @return
     */
    int getWidth() const { return this->width_; }

private:
    int length_; //Length

    int width_; //Width

    double radius_; // all circles have the same radius
};



namespace mrflow{
    namespace cfree {


    class ConfigurationFreeSpace : public Geometry {

        public:
            std::string maps_path_;
            std::string map_file_;

            //Class initizalied by the file path where the polygons are defined
            ConfigurationFreeSpace(const std::string filename);
            ConfigurationFreeSpace(){};
            ConfigurationFreeSpace(const ConfigurationFreeSpace &orig);
            virtual ~ConfigurationFreeSpace(){};




            //*****************************************************************************

        /**
         * Get the polygon as defined in the file "filename"
         * @param polyIndex
         * @return
         */
        Polygon getPolygon(int polyIndex) const;


        /**
             * Get original amount of polygons in file "filename"
             * @return
             */
            int getNumberPolygons() const { return this->_polygonSet.size(); }



            /**
             * Get adjancent polygons to polygon with id polyId
             * @param polyId
             * @return
             */
            std::vector<int> getConnectedPolygons(int polyId) const { return this->_connectivityMap.at(polyId); };

//            /**
//             * Are polygons connected/adjacent
//             * @param polyId_1
//             * @param polyId_2
//             * @return
//             */
//            bool arePolygonsConnected(int polyId_1, int polyId_2);



            /**
             * Return if point (x,y) is in polygon with id polygonId
             * @param polygonId
             * @param x
             * @param y
             * @return
             */
            bool isInPolygon(int polygonId, double x, double y);



            /**
             * Include the vehicle footprint in class cfree
             * @param footprint
             */
            void addVehicleFootprint(VehicleFootprint *footprint) { this->footprint_ = footprint; }

            /**
             * Return vehicle footprint
             * @return
             */
            VehicleFootprint *getVehicleFootprint() const { return this->footprint_; }


            /**
             * Checks if polygon poly1 is already in cfree
             * @param poly1
             * @param polygonIds
             * @return
             */
            bool isPolygonInCfree(const Polygon &poly1, std::vector<int> polygonIds);

            /**
             * Creates a vehicle footprint polygon at (x,y,theta)
             * @param x
             * @param y
             * @param theta
             * @return
             */
            Polygon createFootprintPolygon(int x, int y, double theta);

        /**
        * Sample xy part of state inside polygon over a gaussian
        * State has to have a config space in xy (R2)
        * @param poly
        * @param state
        */
        // void samplePolygon(const Polygon &poly, ob::State *state);



        /**
        * Roughly approximates the maximum number of robots that fit in a polygon
        * By computing maxside of a polygon and how many lengths fit in there
        * @param poly
        * @return
        */
        int maxNumberOfRobotsInPolygon(const Polygon &poly);


        /**
             * Return filename of original polgyons
             * @return
             */
            std::string getFilename() const{return this->fileName;};


        protected:
            //Filename with the original polygons from file
            std::string fileName;

            //Set of Polygons
            PolygonSet _polygonSet;

            //Topological graph of the Cfree
            PolygonConnectivityGraph _connectivityMap;

            //Topological graph Extraction/computation utility
            gtl::connectivity_extraction_90<int> _polygonConnectivityAlgorithm;
            //gtl::connectivity_extraction_45<int>  other extraction algorithm;


            //Vehicle/Robot Footprint
            VehicleFootprint *footprint_;




            void addPolygon(const Polygon &poly);

            /**
             * Generates the topological graph of the added polygons or i.e. of cfree
             */
            void createConnectivityMap();

            /**
             * Reads the polygons from file "filename" and saves them in _polygonSet
             * @param filename
             */
            void readPolygonsFromFile(std::string filename);







        };

    }
}
#endif /* CONFIGURATIONFREESPACE_H */

