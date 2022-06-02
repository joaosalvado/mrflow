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
        struct PoligonsInfo
        {
            std::vector<int> maxNumRobots;
            std::vector<ConfigurationFreeSpace::Point> center;
        };

        struct Obstacle{
            // Ellipse
            double major_axis, minor_axis;
            ConfigurationFreeSpace::Point center;
            std::shared_ptr<ConfigurationFreeSpace::Polygon> bb_i; //bounding box inside ellipse
            std::shared_ptr<ConfigurationFreeSpace::Polygon> bb_o; //bounding box outside ellise
        };

        struct OfreeBit {
            mrflow::cfree::Geometry::Polygon convexhull;
            std::vector<std::shared_ptr<mrflow::cfree::Obstacle>> obstacles;
        };


        class SimpleCfree : public MetaConfigurationFreeSpace
        {
        public:
            std::vector<std::shared_ptr<Obstacle>> obstacles;
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
            void plotObstacles(String file);
            void plotRectangles(PolygonSet &ps);
            void plotMultirobotPath(
                    std::vector<std::vector<int>> path,
                    String file);
            void plotPath(std::vector<int> path, int robot_id);
            void loadMap(String file);
            PoligonsInfo polygon_info_;
            std::vector<std::vector<double>> transition_cost_matrix_;

            void createConnectivityGraph(){
                this->createMetaConnectivityMap_AdjacentPolygons();
                this->computePolygonsInfo();
                this->setPolygonTransitionCosts();
                // this->updateConnectivityGraphWithObstacles();
            }
            void updateConnectivityGraphWithObstacles();
            void addMrenvPolygons(
                    std::list<std::shared_ptr<mrenv::Tesselation::Rectangle>> &rects);
            void addObstacles(std::vector<std::vector<cv::Point> > obstacles);
            std::vector<std::shared_ptr<cfree::Geometry::Polygon>> getObstaclesBBo();

            //void createSquareCoverage(std::string maps_path, std::string map_file, double scale);
            //void plotCoverage() { tessel.plotBestCover(); };
            void line_color(cv::Mat img, cv::Point start, cv::Point end, int r);
            void line(cv::Mat img, cv::Point start, cv::Point end);
            void writeText(cv::Mat img, cv::Point poin, const char *message);
            void addFilledCircle(cv::Mat img, cv::Point center, int r, int radius);
            void fillPolygon(Mat img, const cv::Point *points, int n_pts);
            void addFillPolygon(Mat img, Polygon pol);
            cv::Mat getNewImage(String file );
        private:
            cv::Mat img;


            std::vector<cv::Scalar> colors =
                    {
                            Scalar(230, 25, 75), Scalar(60, 180, 75), Scalar{255, 225, 25},
                            Scalar{0, 130, 200},Scalar(240, 50, 230),
                            Scalar(210, 245, 60), Scalar(250, 190, 212), Scalar(0, 128, 128),
                            Scalar(220, 190, 255), Scalar(170, 110, 40), Scalar(255, 250, 200),
                            Scalar(128, 0, 0), Scalar(170, 255, 195), Scalar(128, 128, 0),
                            Scalar(255, 215, 180), Scalar(0, 0, 128),Scalar(128, 128, 128),
                            Scalar(0, 0, 64), Scalar(0, 64, 0), Scalar(64, 0, 0),
                            Scalar(0, 64, 64), Scalar(64, 0, 64), Scalar(64, 64, 0),
                            Scalar(0, 0, 192), Scalar(0, 192, 0), Scalar(192, 0, 0),
                            Scalar(0, 192, 192), Scalar(192, 0, 192), Scalar(192, 192, 0),
                            Scalar(0, 0, 0)
                    };
            //mrenv::Tesselation tessel;
            //void convexPolygon(cv::Mat img, const cv::Point *points, int n_pts);

            bool robotFitsPolygon(
                    const std::vector<std::shared_ptr<Polygon>> &obstacles,
                    Polygon polygon, Polygon &polygon_no_obstacles );

        };
    } // namespace cfree
} // namespace mrrm

#endif