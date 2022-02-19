/*
 * Created Date: Tuesday, May 5th 2020, 11:26:42 pm 
 * Author: Guangda Chen
 * 
 * Copyright (c) 2020 DRL_NAV
 */

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

/*
 GridMap类表示格子地图，包括全局静态地图和机器人局部视角地图
*/
class GridMap
{
public:
    GridMap();
    GridMap& operator =(const GridMap & grid_map);
    GridMap(const GridMap& grid_map);
    // 从图片文件加载静态地图
    void read_image(std::string file_name, double view_map_resolution);
    // 地图世界坐标系和地图格子索引的转换
    void world2map(Point2d xy, Point2i &mn);
    void map2world(Point2i mn, Point2d& xy);

    bool is_in_map(Point2i mn);
    void empty_map();
    double resolution_; // 地图格子分辨率
    int img_width_; // 宽
    int img_height_; // 高
    Mat map_; // 地图
};
