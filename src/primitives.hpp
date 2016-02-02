#ifndef primitives_hpp
#define primitives_hpp

#include "vsr/space/vsr_cga2D.h"
#include "eggs/variant.hpp"
#include <array>

namespace planar {

	typedef vsr::cga2D::Vec Point2d;
	typedef vsr::cga2D::Vec Vec2d;

	struct LineSegment{
		std::array<Point2d, 2> pts;
	};

	struct Circle{
		Point2d center;
		float radius;
	};

	struct Arc{
		Circle circle;
		LineSegment endpoints;
	};

	Arc ArcWithDirectionAndAngle(const Point2d &center, float radius, const vsr::cga2D::Vec &direction, float angle);

	LineSegment Offset(const LineSegment &segment, float amt);
	Circle Offset(const Circle &circle, float amt);
	Arc Offset(const Arc &arc, float amt);

	std::vector<Vec2d> Tangents(const LineSegment &segment);
	std::vector<Vec2d> Tangents(const Circle &circle);
	std::vector<Vec2d> Tangents(const Arc &arc);
	
	std::vector<Vec2d> Endpoints(const LineSegment &segment);
	std::vector<Vec2d> Endpoints(const Circle &circle);
	std::vector<Vec2d> Endpoints(const Arc &arc);

	std::vector<Point2d> Intersect(const LineSegment &segment1, const LineSegment &segment2);
	std::vector<Point2d> Intersect(const Circle &circle1, const Circle &circle2);
	std::vector<Point2d> Intersect(const Arc &arc1, const Arc &arc2);
	std::vector<Point2d> Intersect(const Circle &circle, const LineSegment &segment);
	std::vector<Point2d> Intersect(const LineSegment &segment, const Circle &circle);
	std::vector<Point2d> Intersect(const LineSegment &segment, const Arc &arc);
	std::vector<Point2d> Intersect(const Arc &arc, const LineSegment &segment);
	std::vector<Point2d> Intersect(const Circle &circle, const Arc &arc);
	std::vector<Point2d> Intersect(const Arc &arc, const Circle &circle);


	class Curve {
	  public:
		enum CurveType{
			LineSegment = 0,
			Circle,
			Arc
		};
		
		template <typename T>
		Curve(T x) : self_(new Model<T>(move(x))) {}
		
		Curve(const Curve& x) : self_(x.self_->copy_()) {}
		Curve(Curve&&) noexcept = default;
		
		Curve& operator=(const Curve& x) {
			Curve tmp(x);
			*this = move(tmp);
			return *this;
		}
		
		Curve& operator=(Curve&&) noexcept = default;
		
		friend Curve Offset(const Curve& x, float offset) {
			return x.self_->Offset_(offset);
		}
		
		friend std::vector<Vec2d> Tangents(const Curve& x) {
			return x.self_->Tangents_();
		}
		
		friend std::vector<Vec2d> Endpoints(const Curve& x) {
			return x.self_->Endpoints_();
		}
		
		friend const void* Target(const Curve& x) {
			return x.self_->Target_();
		}
		
		friend CurveType TargetType(const Curve& x) {
			return x.self_->TargetType_();
		}
		
		
		friend std::vector<Point2d> Intersect(const Curve& x, const Curve& y) {
			return x.self_->Intersect_(y);
		}
		
	  private:
		struct Concept {
			virtual ~Concept() = default;
			virtual Concept* copy_() const = 0;
			virtual Curve Offset_(float offset) const = 0;
			virtual std::vector<Vec2d> Tangents_() const = 0;
			virtual std::vector<Vec2d> Endpoints_() const = 0;
			virtual const void* Target_() const = 0;
			virtual CurveType TargetType_() const = 0;
			virtual std::vector<Point2d> Intersect_(const Curve &rhs) const = 0;
			// Visitor pattern methods
			virtual std::vector<Point2d> Intersect_(const struct LineSegment &rhs) const = 0;
			virtual std::vector<Point2d> Intersect_(const struct Circle &rhs) const = 0;
			virtual std::vector<Point2d> Intersect_(const struct Arc &rhs) const = 0;
		};
		
		template<typename T>
		struct CurveTraits{};
		
		
		
		template <typename T>
		struct Model : Concept {
			Model(T x) : data_(move(x)) { }
			Concept* copy_() const { return new Model(*this); }
			Curve Offset_(float offset) const  {
				return Offset(data_, offset);
			}
			std::vector<Vec2d> Tangents_() const {
				return Tangents(data_);
			}
			std::vector<Vec2d> Endpoints_() const {
				return Endpoints(data_);
			}
			const void* Target_() const {
				return &data_;
			}
			CurveType TargetType_() const {
				return CurveTraits<T>::value;
			}
			
			std::vector<Point2d> Intersect_(const Curve &rhs) const {
				return rhs.self_->Intersect_(data_);
			}
			std::vector<Point2d> Intersect_(const struct LineSegment &rhs) const {
				return Intersect(data_, rhs);
			}
			std::vector<Point2d> Intersect_(const struct Circle &rhs) const {
				return Intersect(data_, rhs);
			}
			std::vector<Point2d> Intersect_(const struct Arc &rhs) const {
				return Intersect(data_, rhs);
			}
			
			T data_;
		};
		
	   unique_ptr<const Concept> self_;
	};

	Curve Offset(const Curve& x, float offset);
	std::vector<Vec2d> Tangents(const Curve& x);
	std::vector<Vec2d> Endpoints(const Curve& x);

	template<>
	struct Curve::CurveTraits<struct LineSegment> {
		static const Curve::CurveType value = Curve::CurveType::LineSegment;
	};
	
	template<>
	struct Curve::CurveTraits<struct Circle> {
		static const Curve::CurveType value = Curve::CurveType::Circle;
	};
	
	template<>
	struct Curve::CurveTraits<struct Arc> {
		static const Curve::CurveType value = Curve::CurveType::Arc;
	};
}

#endif
