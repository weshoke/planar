#include "primitives.hpp"
#include "vsr/space/vsr_cga2D_op.h"
#include <cmath>
#include <limits>

namespace planar {

	vsr::cga2D::Line ToLine(const LineSegment &segment) {
		return vsr::cga2D::Construct::point(segment.pts[0]) ^ vsr::cga2D::Construct::point(segment.pts[1]) ^ vsr::cga2D::Infinity(1.f);
	}

	vsr::cga2D::Dls ToDualCircle(const Circle &circle) {
		return vsr::nga::Round::dls(circle.center, std::abs(circle.radius));
	}

	vsr::cga2D::Vec Normalize(const vsr::cga2D::Vec &v) {
		return v / v.norm();
	}

	vsr::cga2D::Vec LineSegmentDirection(const LineSegment &segment) {
		return Normalize(segment.pts[1] - segment.pts[0]);
	}

	vsr::cga2D::Vec RotateCCW(const vsr::cga2D::Vec &dir) {
		return vsr::cga2D::Vec(-dir[1], dir[0]);
	}

	vsr::cga2D::Vec RotateCW(const vsr::cga2D::Vec &dir) {
		return vsr::cga2D::Vec(dir[1], -dir[0]);
	}

	bool LineSegmentContainsPoint(const LineSegment &segment, const vsr::cga2D::Vec &pt) {
		auto segment_dir = segment.pts[1] - segment.pts[0];
		auto to_pt0 = pt - segment.pts[0];
		auto to_pt1 = pt - segment.pts[1];
		
		auto along0 = segment_dir <= to_pt0;
		auto along1 = segment_dir <= to_pt1;
		return along0[0] >= 0.f && along1[0] <= 0.f;
	}


	LineSegment Offset(const LineSegment &segment, float amt) {
		// Normal to line segment where amt > 0 is a translation
		// in the direction of CCW(Dir(segment)) and amt < 0 is
		// is a translation in the direction of CW(Dir(segment))
		auto dir = LineSegmentDirection(segment);
		auto offset_dir = amt >= 0.f ? RotateCCW(dir) : RotateCW(dir);
		return LineSegment{segment.pts[0] + offset_dir, segment.pts[1] + offset_dir};
	}

	Circle Offset(const Circle &circle, float amt) {
		// Can return circles with negative radius
		auto radius = circle.radius + amt;
		// If the sign of the radius changes, the circle disappears.
		// This is signalled by setting the returned Circle's radius to NaN.
		if(std::signbit(radius) != std::signbit(circle.radius)) {
			radius = std::numeric_limits<float>::quiet_NaN();
		}
		return Circle{circle.center, radius};
	}

	Arc Offset(const Arc &arc, float amt) {
		auto circle = Offset(arc.circle, amt);
		if(std::isnan(circle.radius)) {
			return Arc{circle, LineSegment{circle.center, circle.center}};
		}
		
		auto dir0 = Normalize(arc.endpoints.pts[0] - arc.circle.center);
		auto dir1 = Normalize(arc.endpoints.pts[1] - arc.circle.center);
		auto endpoints = LineSegment{arc.endpoints.pts[0] + dir0, arc.endpoints.pts[0] + dir1};
		return Arc{circle, endpoints};
	}


	std::vector<vsr::cga2D::Vec> Intersect(const LineSegment &segment1, const LineSegment &segment2) {
		auto L1 = ToLine(segment1);
		auto L2 = ToLine(segment2);
		// Intersection as a flat point (Flp)
		auto intersection = (L1.dual() ^ L2.dual()).dual();
		
		// Check if the only intersection is the point at infinity
		if(std::abs(intersection[2]) <= 1e-6) {
			return std::vector<vsr::cga2D::Vec>{pt};
		}
		
		// Check if the intersection point is withint the line segments
		auto pt = vsr::cga2D::Vec(intersection[0], intersection[1]) / intersection[2];
		auto within1 = LineSegmentContainsPoint(segment1, pt);
		auto within2 = LineSegmentContainsPoint(segment2, pt);
		if(!within1 || !within2) {
			return std::vector<vsr::cga2D::Vec>{pt};
		}
		return std::vector<vsr::cga2D::Vec>{pt};
	}

	std::vector<vsr::cga2D::Vec> Intersect(const Circle &circle, const LineSegment &segment) {
		auto C = ToDualCircle(circle);
		auto L = ToLine(segment);
		auto intersection = C <= L;
		auto size = vsr::nga::Round::size(intersection, false);
		
		// Point pair size is negative, no intersection points
		if(size < -1e-6f) {
			return std::vector<vsr::cga2D::Vec>{};
		}
		
		// Get the intersection points
		auto split_pts = vsr::nga::Round::split(intersection);
		auto pt1 = vsr::cga2D::Vec(split_pts[0][0], split_pts[0][1]);
		auto pts = std::vector<vsr::cga2D::Vec>{};
		if(LineSegmentContainsPoint(segment, pt1)) {
			pts.push_back(pt1);
		}
		
		// If the size of the point pair is above threshold, there
		// are 2 valid intersection points
		if(size > 1e-6) {
			auto pt2 = vsr::cga2D::Vec(split_pts[1][0], split_pts[1][1]);
			if(LineSegmentContainsPoint(segment, pt2)) {
				pts.push_back(pt2);
			}
		}
		
		return pts;
	}

	std::vector<vsr::cga2D::Vec> Intersect(const LineSegment &segment, const Circle &circle) {
		return Intersect(circle, segment);
	}

	std::vector<vsr::cga2D::Vec> Intersect(const LineSegment &segment, const Arc &arc) {
		auto dir0 = arc.endpoints.pts[0] - arc.circle.center;
		auto dir1 = arc.endpoints.pts[1] - arc.circle.center;
		auto op = (dir0 ^ dir1)[0];
		auto op_sign = std::signbit(op);
		auto r_sign = std::signbit(arc.circle.radius);
		//auto op_r_different = op_sign ^ r_sign;
		
		// The test is, keep a point if:
		// r > 0 && op > 0 && op0 > 0 && op1 > 0
		// r > 0 && op < 0 && !(op0 < 0 && op1 < 0)
		// r < 0 && op > 0 && !(op0 > 0 && op1 > 0)
		// r < 0 && op < 0 && op0 < 0 && op1 < 0
		
		// TODO: convert test to flags with bits for signs
		auto pts = std::vector<vsr::cga2D::Vec>{};
		auto candidate_pts = Intersect(arc.circle, segment);
		for(const auto& pt : candidate_pts) {
			auto dir = pt - arc.circle.center;
			auto op0 = (dir0 ^ dir)[0];
			auto op1 = (dir ^ dir1)[0];
			auto op0_sign = std::signbit(op0);
			auto op1_sign = std::signbit(op1);
			auto test_pos = !(op_sign || op0_sign || op1_sign);
			auto test_neg = op_sign && op0_sign && op1_sign;
			auto test_pos_pos = !r_sign && test_pos;
			auto test_pos_neg = !r_sign && op_sign && !(op0_sign && op1_sign);
			auto test_neg_pos = r_sign && !op_sign && (op0_sign || op1_sign);
			auto test_neg_neg = r_sign && test_neg;
			std::cout << r_sign << " " << op_sign << " " << op0_sign << " " << op1_sign << "\n";
			std::cout << "\t" << test_pos << " " << test_neg << "\n";
			std::cout << "\t" << test_pos_pos << " " << test_pos_neg << "\n";
			if(test_pos_pos || test_pos_neg || test_neg_pos || test_neg_neg) {
			//if(test_pos_pos ||test_neg_neg || ) {
				pts.push_back(pt);
				std::cout << "ADD: " << pt << "\n";
			}
			else {
				std::cout << "REM: " << pt << "\n";
			}
			std::cout << "\n";
		}
		return pts;
	}

	std::vector<vsr::cga2D::Vec> Intersect(const Arc &arc, const LineSegment &segment) {
		return Intersect(segment, arc);
	}
}