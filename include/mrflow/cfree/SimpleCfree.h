/**
 * @file SimpleCfree.h
 * @author your name (you@domain.com)
 * @brief This class is metacfree without clutter
 *          use metapolygons meta everything variables to be utilized by other solvers
 * @version 0.1
 * @date 2021-01-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SIMPLE_CFREE_H
#define SIMPLE_CFREE_H

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "mrflow/cfree/MetaConfigurationFreeSpace.h"
#include <iostream>

//Mrenv
#include "mrenv/Tesselation.h"


namespace mrflow{
    namespace cfree
    {
        typedef struct PoligonsInfo
        {
            std::vector<int> maxNumRobots;
            std::vector<mrflow::cfree::ConfigurationFreeSpace::Point> center;
        } PoligonsInfo;

        class SimpleCfree : public MetaConfigurationFreeSpace
        {
        public:
            double px_mm = 1, mm_px = 1, m2mm;
            SimpleCfree(const std::string filename)
                : MetaConfigurationFreeSpace(filename) {}
            SimpleCfree(double length, double width, double px2m_)
                : MetaConfigurationFreeSpace(){
                this->px2m = px2m_;
                auto footprint = std::make_shared<VehicleFootprint>(
                        length, width);
                this->addVehicleFootprint(footprint);
            };
            virtual ~SimpleCfree(){};

            virtual void computePolygonsInfo();

            virtual void setPolygonTransitionCosts();

            std::vector<std::vector<int>> getConnectivity()
            {
                return this->_metaConnectivityMap;
            }

            void plotCfree();
            void plotRectangles(PolygonSet &ps);

            PoligonsInfo polygon_info_;
            std::vector<std::vector<double>> transition_cost_matrix_;

            void createConnectivityGraph(){
                this->createMetaConnectivityMap_AdjacentPolygons();
                this->computePolygonsInfo();
                this->setPolygonTransitionCosts();
            }
            void addMrenvPolygons(
                    std::list<std::shared_ptr<mrenv::Tesselation::Rectangle>> &rects);
            //void createSquareCoverage(std::string maps_path, std::string map_file, double scale);
            //void plotCoverage() { tessel.plotBestCover(); };

            void line(cv::Mat img, cv::Point start, cv::Point end);
            void writeText(cv::Mat img, cv::Point poin, const char *message);
            void fillPolygon(Mat img, const cv::Point *points, int n_pts);
        private:
            double px2m;
            //mrenv::Tesselation tessel;
            //void convexPolygon(cv::Mat img, const cv::Point *points, int n_pts);

        };
    } // namespace cfree
} // namespace mrrm

#endif