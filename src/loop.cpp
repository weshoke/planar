#include "loop.hpp"
#include <typeinfo>

namespace planar {

uint32_t FlagFromVariantIndex(size_t idx) {
	return idx << 30;
}

Loop Loop::Create(const std::vector<Curve> &curves) {
	auto loop = Loop{};
	// TODO: use enumerate function
	int i = 0;
	for(const auto& c : curves) {
		auto idx = c.which();
		switch(idx) {
			// TODO: possible to generate enums from tuple positions at compile time?
			case 0: loop.line_segments_.push_back(*c.template target<LineSegment>()); break;
			case 1: loop.circles_.push_back(*c.template target<Circle>()); break;
			case 2: loop.arcs_.push_back(*c.template target<Arc>()); break;
		}
		
		auto flag = FlagFromVariantIndex(idx);
		loop.elements_.push_back(i | flag);
		++i;
	}
	return loop;
}

Loop Loop::Offset(float amt) {
	//std::vector<Curve> offset_curves(curves_.size());
	/*
	std::transform(curves_.begin(), curves_.end(), offset_curves.begin(), [amt](const Curve &curve) {
		return Offset(curve, amt);
	});
	*/
	
	// Make quadtree of offset curves
	//
	
	//return Create(offset_curves);
}


Loop::Loop() {}

}