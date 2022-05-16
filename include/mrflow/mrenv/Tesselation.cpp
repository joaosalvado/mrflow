#include "Tesselation.h"
#include <random>
#include <algorithm>
using namespace cv;

void mrenv::Tesselation::addLine(Mat img, Point start, Point end)
{
        int thickness = 2;
        int lineType = LINE_8;
        line(img,
             start,
             end,
             Scalar(255, 0, 0),
             thickness,
             lineType);
}

void mrenv::Tesselation::addFilledCircle(Mat img, Point center)
{
        circle(img,
               center,
               5,
               Scalar(0, 0, 255),
               FILLED,
               LINE_8);
}

void mrenv::Tesselation::addRectangle(Mat img, Point corner1, Point corner2)
{
        cv::rectangle(img,
                      corner1,
                      corner2,
                      Scalar(100, 100, 50));
}

void mrenv::Tesselation::fillRectangle(Mat img, Point corner1, Point corner2)
{
        cv::rectangle(img,
                      corner1,
                      corner2,
                      Scalar(0, 255, 0),
                      -1);
}

void mrenv::Tesselation::whiteRectangle(Mat img, Point corner1, Point corner2)
{
        cv::rectangle(img,
                      corner1,
                      corner2,
                      Scalar(255, 255, 255),
                      -1);
}

void mrenv::Tesselation::blueRectangle(Mat img, Point corner1, Point corner2)
{
        cv::rectangle(img,
                      corner1,
                      corner2,
                      Scalar(30, 210, 100),
                      4);
}

void mrenv::Tesselation::addConvexPolygon(Mat img, const Point *points, int n_pts)
{
        fillPoly(img,
                 &points,
                 &n_pts,
                 1,
                 Scalar(100, 100, 50),
                 LINE_8);

        polylines(img,
                  &points,
                  &n_pts,
                  1,
                  true,
                  Scalar(240, 50, 100));
}

void mrenv::Tesselation::fillPolygonBlack(Mat img, const Point *points, int n_pts)
{
        fillPoly(img,
                 &points,
                 &n_pts,
                 1,
                 Scalar(0, 255, 0),
                 LINE_8);
}

bool mrenv::Tesselation::isColliding(const Point2d &left_bottom_corner, const Point2d &right_upper_corner)
{
        int min_col = left_bottom_corner.x;
        int max_col = right_upper_corner.x;
        int min_row = left_bottom_corner.y;
        int max_row = right_upper_corner.y;

        for (int row = min_row; row < max_row; ++row)
        {
                for (int col = min_col; col < max_col; ++col)
                {
                        auto pixel = gray_img.at<uchar>(row, col);
                        if (pixel == 0)
                        { //black pixel
                                return true;
                        }
                }
        }
        return false;
}

void mrenv::Tesselation::createRectangle(
    const Point2d &center,
    int pos_x, int neg_x,
    int pos_y, int neg_y,
    std::shared_ptr<mrenv::Tesselation::Rectangle> &rect)
{
        rect->left_bottom_corner = Point2d(center.x - neg_x, center.y - neg_y);
        rect->right_upper_corner = Point2d(center.x + pos_x, center.y + pos_y);
}

std::shared_ptr<mrenv::Tesselation::Rectangle> mrenv::Tesselation::maxRectangle(
    int seed_x, int seed_y)
{
        int min_square = this->length_px;
        int pixel_increment = 1;
        auto rect = std::make_shared<Rectangle>();

        int pos_x = min_square;
        int neg_x = min_square;
        int pos_y = min_square;
        int neg_y = min_square;
        //Test Minimal Rectangle 10 x10
        this->createRectangle(Point2d(seed_x, seed_y), pos_x, neg_x, pos_y, neg_y, rect);
        if (this->isColliding(rect->left_bottom_corner, rect->right_upper_corner))
                return nullptr;

        bool pos_x_max = false;
        bool neg_x_max = false;
        bool pos_y_max = false;
        bool neg_y_max = false;
        while (!(pos_x_max && neg_x_max && pos_y_max && neg_y_max))
        {
                if (!pos_x_max)
                        pos_x += pixel_increment;
                if (!neg_x_max)
                        neg_x += pixel_increment;
                if (!pos_y_max)
                        pos_y += pixel_increment;
                if (!neg_y_max)
                        neg_y += pixel_increment;

                createRectangle(Point2d(seed_x, seed_y), pos_x, neg_x, pos_y, neg_y, rect);
                if (!this->isColliding(rect->left_bottom_corner, rect->right_upper_corner))
                {
                        continue;
                }
                else
                { //It is colliding
                        //backtrack
                        if (!pos_x_max)
                                pos_x -= pixel_increment;
                        if (!neg_x_max)
                                neg_x -= pixel_increment;
                        if (!pos_y_max)
                                pos_y -= pixel_increment;
                        if (!neg_y_max)
                                neg_y -= pixel_increment;
                        ;
                        //trial and error
                        pos_x += pixel_increment;
                        createRectangle(Point2d(seed_x, seed_y), pos_x, neg_x, pos_y, neg_y, rect);
                        if (this->isColliding(rect->left_bottom_corner, rect->right_upper_corner))
                        {
                                pos_x -= pixel_increment;
                                pos_x_max = true;
                        }
                        neg_x += pixel_increment;
                        createRectangle(Point2d(seed_x, seed_y), pos_x, neg_x, pos_y, neg_y, rect);
                        if (this->isColliding(rect->left_bottom_corner, rect->right_upper_corner))
                        {
                                neg_x -= pixel_increment;
                                neg_x_max = true;
                        }
                        pos_y += pixel_increment;
                        createRectangle(Point2d(seed_x, seed_y), pos_x, neg_x, pos_y, neg_y, rect);
                        if (this->isColliding(rect->left_bottom_corner, rect->right_upper_corner))
                        {
                                pos_y -= pixel_increment;
                                pos_y_max = true;
                        }
                        neg_y += pixel_increment;
                        createRectangle(Point2d(seed_x, seed_y), pos_x, neg_x, pos_y, neg_y, rect);
                        if (this->isColliding(rect->left_bottom_corner, rect->right_upper_corner))
                        {
                                neg_y -= pixel_increment;
                                neg_y_max = true;
                        }
                }
        }

        this->fillRectangle(gray_img, rect->left_bottom_corner, rect->right_upper_corner);

        return rect;
}

void mrenv::Tesselation::coverRectangles()
{
        int T = 300;
        std::random_device rd;                                            // obtain a random number from hardware
        std::mt19937 gen(rd());                                           // seed the generator
        std::uniform_int_distribution<int> distr_x(0, gray_img.cols - 1); // define the range
        std::uniform_int_distribution<int> distr_y(0, gray_img.rows - 1); // define the range
        std::priority_queue<std::shared_ptr<Rectangle>, std::vector<std::shared_ptr<Rectangle>>, compareRectangle> rectangles;

        this->best_cover_ = std::make_shared<cover>();
        for (int t = 0; t < T; ++t)
        {

                std::binomial_distribution<int> change_end(2, t / T); //higher change to be 1 in end
                //Create a cover
                //cover curr_cover;

                for (int n = 0; n < 200; ++n)
                {
                        auto rect = maxRectangle(distr_x(gen), distr_y(gen));
                        if (rect != nullptr)
                        {
                                //this->best_cover_->rectangles.push_back(rect);
                                rectangles.push(rect);
                        }
                }

                double prob = std::log(((T - t) / T));
                std::binomial_distribution<int> remove_pol_prob(1, prob); //higher change to be 1 in begin

                while (remove_pol_prob(gen))
                {
                        if (!rectangles.empty())
                        {
                                this->whiteRectangle(gray_img, rectangles.top()->left_bottom_corner, rectangles.top()->right_upper_corner);
                                rectangles.pop();
                        }
                        else
                        {
                                break;
                        }
                }
        }

        //Save rectangles in best cover
        while (!rectangles.empty())
        {
                best_cover_->rectangles.push_back(rectangles.top());
                rectangles.pop();
        }
        best_cover_->area = this->computeCoverArea(*best_cover_);
        best_cover_->num_pol = best_cover_->rectangles.size();
}

double mrenv::Tesselation::computeCoverArea(const cover &cov)
{
        double area_ = 0.0;
        for (auto &rect : cov.rectangles)
        {
                area_ += area(*rect);
        }
        return area_;
}

void mrenv::Tesselation::plotBestCover()
{
    //  Plot cover
        for (auto &rect : this->best_cover_->rectangles)
        {
                this->blueRectangle(color_img, rect->left_bottom_corner, rect->right_upper_corner);
                //this->fillRectangle(gray_img, rect->left_bottom_corner, rect->right_upper_corner);
        }
        // Plot robot for scale
        auto robot = std::make_shared<Rectangle>();
        int pos_y = this->length_px*0.5;
        int neg_y = this->length_px*0.5;
        int pos_x = this->width_px*0.5;
        int neg_x = this->width_px*0.5;
        //Test Minimal Rectangle 10 x10
        this->createRectangle(Point2d(this->length_px, this->width_px), pos_x, neg_x, pos_y, neg_y, robot);
        this->fillRectangle(color_img, robot->left_bottom_corner, robot->right_upper_corner);


        std::cout << "[Tesselation] " << this->best_cover_->rectangles.size() << " polygons" << std::endl;
        imshow("Display win.dow1", gray_img);
        imshow("Display win.dow2", color_img);
        waitKey();
}


double mrenv::Tesselation::area(const Rectangle &rect)
{
        return (rect.left_bottom_corner.x - rect.right_upper_corner.x) *
                   (rect.left_bottom_corner.x - rect.right_upper_corner.x) +
               (rect.left_bottom_corner.y - rect.right_upper_corner.y) *
                   (rect.left_bottom_corner.y - rect.right_upper_corner.y);
}

void mrenv::Tesselation::doubleImage()
{
        Mat result1, result2, result3;
        vconcat(this->color_img, this->color_img, result1);
        resize(result1, result2, Size(this->color_img.cols, this->color_img.rows));

        int top = 2;
        int bottom = top;
        int left = 2;
        int right = left;
        copyMakeBorder(result2, result3, top, bottom, left, right, BORDER_CONSTANT, 2);
        imwrite("qr4.png", result2);
        imwrite("qrb4.png", result3);
        waitKey();
}