#ifndef LIDAR_CALIBRATION
#define LIDAR_CALIBRATION

#include "lidar_information.h"
#include <math.h>
#include <vector>

using namespace std;

const int map_ratio = 40;

struct Vector2D {
    double x;
    double y;

    double Length() {
        return sqrt(x * x + y * y);
    }

    double Dot(Vector2D v) {
        return x * v.x + y * v.y;
    }

    double Cross(Vector2D v) {
        return x * v.y - y * v.x;
    }

    double Angle(Vector2D v) {
        double dot = Dot(v);
        double cross = Cross(v);
        return atan2(cross, dot);
    }
};

struct LineSegment {
    Vector2D p1;
    Vector2D p2;
};


typedef struct 
{
    float x,y;
    float theta;
}pos_t;

typedef struct _point_
{
    float x , y , q ;
    int index ;
    float angle;
    float range;
    bool change = false;
    uint16_t intensity;
}point_t;

typedef struct _point_cloud_
{
    std::vector<point_t> data;
    pos_t origin;
    point_t origin_point;
}pcloud_t;

struct point_lines{
  point_t line_start;
  point_t line_end;
};

typedef struct _border_frame_
{
    int top,bottom;
    int left,right;
}border_frame_t;

class coord_tt
{
    public:
    int x,y;
    float theta;
    coord_tt(int x = 0, int y = 0, float theta = 0.0)
        :x(x), y(y), theta(theta){}
    void move(coord_tt);
    pos_t pos();
};

struct Line_new {
    double slope;
    double intercept;
};

typedef struct grid
{
    int x,y;
    bool operator==(const grid &b)const{return x == b.x && y == b.y;}
    bool operator!=(const grid &b)const{return x != b.x || y != b.y;}
    bool operator<(const grid &b)const {if(x == b.x) return y < b.y; return x < b.x;}
    bool operator>(const grid &b)const {if(x == b.x) return y > b.y; return x > b.x;}
    grid operator+(const grid &b) {return {x + b.x, y + b.y};}
    grid operator-(const grid &b) {return {x - b.x, y - b.y};}
    
    grid () {x = y = 0;}
    grid (int x_, int y_) {x = x_; y = y_;}
    grid (coord_tt coord){x = coord.x; y = coord.y;}
    
    bool near(grid &b)  {return abs(x - b.x) <= 1 && abs(y - b.y) <= 1;}
    //bool vaild() {return x >= 0 && y >= 0 && x < map_width_limit && y < map_width_limit;}
    void move(grid b) {x += b.x; y += b.y;} 
}grid_t;

typedef struct
{
    float dist;
    float ratio;
    grid_t point;
}p2seg_ret_t;


typedef struct _calibration_params_
{
  float distortion_processing_distance = 500;
 
  point_t origin_point;

  int calibration_type = 0;

  int line_judgment_point_num;
}calibration_params_t;

extern calibration_params_t calibration_params;

void lidar_calibration(LaserScan &outscan);

void getLineKB(point_t start_point, point_t end_point, double &k, double &b);

double pointToLineDist(point_t point, double k, double b);

void pcloud_build_by_pos_scan(LaserScan &outscan ,pcloud_t &pcloud);

void fitting_line(pcloud_t &pcloud);

bool line_judgment(std::vector<point_t> line, point_t point);

void neighbor_lines_new(std::vector<point_t> line_first, std::vector<point_t> line_second, pcloud_t &pcloud);

void Point_cloud_straightening_new(std::vector<point_t> line_first, std::vector<point_t> line_second, pcloud_t &pcloud);

float pointTopointDist(point_t a, point_t b);

int isLeft(point_t p, point_t a, point_t b);

void straightening_process_new(std::vector<point_t> line_point, point_t a, point_t b, pcloud_t &pcloud);

bool intersect(LineSegment line1, LineSegment line2, Vector2D &intersection);

point_t calculateIntersection(Line_new line1, Line_new line2);

#endif
