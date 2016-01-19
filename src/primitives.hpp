#ifndef primitives_hpp
#define primitives_hpp

#include "vsr/space/vsr_cga2D.h"
#include "eggs/variant.hpp"

namespace planar {

	typedef vsr::cga2D::Vec Point2D;

	struct LineSegment{
		Point2D pts[2];
	};

	struct Circle{
		Point2D center;
		float radius;
	};

	struct Arc{
		Circle circle;
		LineSegment endpoints;
	};

	typedef eggs::variant<LineSegment, Circle, Arc> Curve;

	LineSegment Offset(const LineSegment &segment, float amt);
	Circle Offset(const Circle &circle, float amt);
	Arc Offset(const Arc &arc, float amt);


	std::vector<Point2D> Intersect(const LineSegment &segment1, const LineSegment &segment2);
	std::vector<Point2D> Intersect(const Circle &circle, const LineSegment &segment);
	std::vector<Point2D> Intersect(const LineSegment &segment, const Circle &circle);
	std::vector<Point2D> Intersect(const LineSegment &segment, const Arc &arc);
	std::vector<Point2D> Intersect(const Arc &arc, const LineSegment &segment);

}

#endif
