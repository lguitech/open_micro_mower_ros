#ifndef slic3r_MultiPoint_hpp_
#define slic3r_MultiPoint_hpp_

#include "libslic3r.h"
#include <algorithm>
#include <vector>
#include "Line.hpp"
#include "Point.hpp"

namespace Slic3r {

class BoundingBox;

class MultiPoint
{
    public:
    Points points;
    
    operator Points() const;
    void scale(double factor);  // 比例缩放
    void translate(double x, double y);  // 平移
    void translate(const Point &vector);  // 平移
    void rotate(double angle);
    void rotate(double angle, const Point &center);
    void reverse();
    Point first_point() const;
    virtual Point last_point() const = 0;
    virtual Lines lines() const = 0;
    double length() const;
    bool is_valid() const { return this->points.size() >= 2; }

    int find_point(const Point &point) const;
    bool has_boundary_point(const Point &point) const;
    BoundingBox bounding_box() const;
    
    // Return true if there are exact duplicates.
    bool has_duplicate_points() const;
    
    // Remove exact duplicates, return true if any duplicate has been removed.
    bool remove_duplicate_points();
    
    void append(const Point &point);
    void append(const Points &points);
    void append(const Points::const_iterator &begin, const Points::const_iterator &end);
    bool intersection(const Line& line, Point* intersection) const;
    std::string dump_perl() const;
    
    static Points _douglas_peucker(const Points &points, const double tolerance);
    
    protected:
    MultiPoint() {};
    explicit MultiPoint(const Points &_points): points(_points) {};
    ~MultiPoint() = default;
};

} // namespace Slic3r

#endif
