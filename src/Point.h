#pragma once
class Point
{
public:
    Point(int _x, int _y) :x(_x), y(_y) {}
    Point() :x(0), y(0) {}
    Point(const Point& point) {
        x = point.x;
        y = point.y;
    }
    double x, y;
};