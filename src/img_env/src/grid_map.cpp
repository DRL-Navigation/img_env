#include "grid_map.h"
#include <math.h>

GridMap::GridMap()
{
}

GridMap& GridMap::operator =(const GridMap & grid_map)
{
    if (this != &grid_map)
    {
        img_width_ = grid_map.img_width_;
        img_height_ = grid_map.img_height_;
        resolution_ = grid_map.resolution_;
        map_ = grid_map.map_.clone();
    }
    return *this;
}

GridMap::GridMap(const GridMap& grid_map)
{
    img_width_ = grid_map.img_width_;
    img_height_ = grid_map.img_height_;
    resolution_ = grid_map.resolution_;
    map_ = grid_map.map_.clone();
}

void GridMap::read_image(std::string file_name, double view_map_resolution)
{
    Mat image = imread(file_name, IMREAD_GRAYSCALE);
    img_width_ = int(image.size().width * resolution_ / view_map_resolution);
    img_height_ = int(image.size().height * resolution_ / view_map_resolution);
    Size s;
    s.height = img_height_;
    s.width = img_width_;
    resize(image, map_, s);
    resolution_ = view_map_resolution;
}

void GridMap::world2map(Point2d xy, Point2i& mn)
{
    mn.x = int(round(xy.x / resolution_));
    mn.y = int(round(xy.y / resolution_));
}

void GridMap::map2world(Point2i mn, Point2d &xy)
{
    xy.x = mn.x * resolution_;
    xy.y = mn.y * resolution_;
}

bool GridMap::is_in_map(Point2i mn)
{
    return mn.x >= 0 && mn.x < img_height_ && mn.y >= 0 && mn.y < img_width_;
}

void GridMap::empty_map()
{
    map_ = cv::Mat::ones(img_height_, img_width_, CV_8UC(1)) * 200;
}
