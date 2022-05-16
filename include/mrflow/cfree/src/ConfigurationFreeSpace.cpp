/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ConfigurationFreeSpace.cpp
 * Author: ohmy
 * 
 * Created on August 17, 2018, 2:56 PM
 */


#include "mrflow/cfree/ConfigurationFreeSpace.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <boost/geometry.hpp>


using namespace mrflow::cfree;

ConfigurationFreeSpace::ConfigurationFreeSpace(const std::string filename) : Geometry() {
    this->fileName = filename;
    this->readPolygonsFromFile(filename);
    this->createConnectivityMap();

}

ConfigurationFreeSpace::ConfigurationFreeSpace(const ConfigurationFreeSpace& orig) : Geometry() {

}


void ConfigurationFreeSpace::readPolygonsFromFile(std::string filename) {
    std::ifstream file;
    std::string line;
    file.open(filename);

    if (!file) {
        std::cerr << "The file cannot be open!" << filename << std::endl;
        return;
    }

    std::list<Point> listPoints{};
    while (std::getline(file, line)) {

        //test for empty lines
        if (line.empty())
            continue;

        //test for comments
        if (line.at(0) == '#')
            continue;


        //remove spaces
        line.erase(std::remove_if(line.begin(),
                line.end(),
                [](unsigned char x) {
                    return std::isspace(x);
                }),
        line.end());


        if (line.compare("P") == 0) {
            listPoints.clear();
        } else if (line.compare("~P") == 0) {
            Polygon poly = this->createPolygon(listPoints);
            this->addPolygon(poly);
        } else {
            //remove parentheses 
            line.erase(std::remove_if(line.begin(),
                    line.end(),
                    [](unsigned char x) {
                        return (x == '(' || x == ')');
                    }),
            line.end());
            //split x and y
            int x, y;
            std::istringstream lineStream(line);
            std::string str;
            getline(lineStream, str, ',');
            std::istringstream xStream(str);
            xStream >> x;
            getline(lineStream, str, ',');
            std::istringstream yStream(str);
            yStream >> y;

            Point point = this->createPoint(x, y);
            listPoints.push_back(std::move(point));
        }


    }

}



void ConfigurationFreeSpace::addPolygon(const Polygon & poly) {
    this->_polygonSet.push_back(std::move(poly));
    // or this->_polygonSet += pol;
}

void ConfigurationFreeSpace::createConnectivityMap() {
    //Insert Polygons for connectivity algorithm
    for (Polygon poly : this->_polygonSet) {
        _polygonConnectivityAlgorithm.insert(gtl::view_as<gtl::polygon_90_concept>(poly));
    }

    std::vector<std::set<int> > graph(this->_polygonSet.size());

    //populate the graph with edge data -> less expensive computation, more difficult to work with
    _polygonConnectivityAlgorithm.extract(graph);

    this->_connectivityMap.resize(this->_polygonSet.size());
    //Change the structure of the graph to a vector of vectors
    int polyId = 0;
    for (std::set<int> adjacencyList : graph) {
        for (auto it = adjacencyList.begin(); it != adjacencyList.end(); ++it) {
            this->_connectivityMap[polyId].push_back(*it);
        }
        ++polyId;
    }


    //make a map type graph to compare results -> more expensive computation, easier to work with
    // this->_connectivityMap = graph;
}

ConfigurationFreeSpace::Polygon ConfigurationFreeSpace::getPolygon(int polyIndex) const {
    return _polygonSet.at(polyIndex);
}


bool ConfigurationFreeSpace::isInPolygon(int polygonId, double x, double y) {
    Polygon poly = this->getPolygon(polygonId);
    Bounds bounds = this->boundaries(poly);

    if (x < bounds.xMin)
        return false;
    if (x > bounds.xMax)
        return false;
    if (y < bounds.yMin)
        return false;
  return y <= bounds.yMax;

}

//bool ConfigurationFreeSpace::arePolygonsConnected(int polyId_1, int polyId_2) {
//    std::for_each(this->_connectivityMap.at(polyId_1).begin(), this->_connectivityMap.at(polyId_1).end(), [&polyId_2](int polyId) {
//        if (polyId_2 == polyId)
//            return true;
//    });
//    return false;
//}

ConfigurationFreeSpace::Polygon ConfigurationFreeSpace::createFootprintPolygon(int x, int y, double theta) {
    int L, W;
    try {
        L = this->footprint_->getLength();
        W = this->footprint_->getWidth();
    } catch (...) {
        std::cout << "Vehicle Footprint was not added";
    }

    Point Corner1 = this->createPoint((int) ((-L / 2) * cos(theta)-(-W / 2) * sin(theta) + x), (int) ((-L / 2) * sin(theta) + (-W / 2) * cos(theta) + y));
    Point Corner2 = this->createPoint((int) ((-L / 2) * cos(theta)-(W / 2) * sin(theta) + x), (int) ((-L / 2) * sin(theta) + (W / 2) * cos(theta) + y));
    Point Corner3 = this->createPoint((int) ((L / 2) * cos(theta)-(W / 2) * sin(theta) + x), (int) ((L / 2) * sin(theta) + (W / 2) * cos(theta) + y));
    Point Corner4 = this->createPoint((int) ((L / 2) * cos(theta)-(-W / 2) * sin(theta) + x), (int) ((L / 2) * sin(theta) + (-W / 2) * cos(theta) + y));

    std::list<Point> corners{Corner1, Corner2, Corner3, Corner4, Corner1};
    return this->createPolygon(corners);
}


bool ConfigurationFreeSpace::isPolygonInCfree(const Polygon &poly1, std::vector<int> polygonIds) {
    PolygonSet p1{poly1}, p2;
    for (int p : polygonIds) {
        p2 += this->getPolygon(p);
    }
    return gtl::area(p1 & p2) != gtl::area(p1);
}



int ConfigurationFreeSpace::maxNumberOfRobotsInPolygon(const Polygon &poly) {
    int Length_Robot = this->footprint_->getLength();
    int Length_Poligon = maximumSide(poly);

    int result = std::floor((0.5 * Length_Poligon - 0.5 * Length_Robot) / Length_Robot);
    return result + 1;

}






/*
void ConfigurationFreeSpace::samplePolygon(const Polygon &poly, ob::State *state) {
    auto *rhs = static_cast<mrrm::state::SingleRobotState::StateType *> (state);

    Bounds bounds = this->boundaries(poly);
    //Trick for x, y to be inside
    bounds.xMax = bounds.xMax - this->footprint_->getLength()/2;
    bounds.xMin = bounds.xMin + this->footprint_->getLength()/2;
    bounds.yMax = bounds.yMax - this->footprint_->getLength()/2;
    bounds.yMin = bounds.yMin + this->footprint_->getLength()/2;



    bool insideBounds = false;
    double x, y;
    while (!insideBounds) {
        int mean = (bounds.xMax - bounds.xMin) / 2 + bounds.xMin;
        int sd = (bounds.xMax - bounds.xMin)*4;
        x = rng_.gaussian((bounds.xMax - bounds.xMin) / 2 + bounds.xMin, (bounds.xMax - bounds.xMin)/4);
        y = rng_.gaussian((bounds.yMax - bounds.yMin) / 2 + bounds.yMin, (bounds.yMax - bounds.yMin)/4);
        x = std::floor(x);
        y = std::floor(y);
        //Enforce Polygon Boundaries
        if (x > bounds.xMin && x < bounds.xMax) {
            if (y > bounds.yMin && y < bounds.yMax)
                insideBounds = true;
        }
    }
    rhs->setXY(x, y);
}
*/







