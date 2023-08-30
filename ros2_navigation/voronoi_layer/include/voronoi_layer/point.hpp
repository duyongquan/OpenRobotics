#ifndef VORONOI_LAYER__POINT_HPP_
#define VORONOI_LAYER__POINT_HPP_

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint 
{
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x,y;
};

#endif  // VORONOI_LAYER__POINT_HPP_
