#include "loop.hpp"
#include <typeinfo>
#include <tuple>
#include "range/v3/view/zip.hpp"
#include "range/v3/algorithm/sort.hpp"
#include "range/v3/range_traits.hpp"

namespace planar {

Loop::Loop(const std::vector<Curve> &curves)
: curves_(curves)
{}

struct PointIntersection{
	uint32_t element_id;
	float param;
};

/*
auto v1 = std::vector<int>{10, 3, 1, 2, 3, 4, 5};
	auto v2 = std::vector<int>{5, 20, 6, 7, 8, 9, 10};
	
	auto zip_range = ranges::view::zip(v1, v2);
	using Rng = decltype(zip_range);
	using CR = ranges::range_common_reference_t<Rng>;
	
	std::cout << typeid(Rng).name() << "\n";
	std::cout << typeid(CR).name() << "\n";
	
	ranges::sort(zip_range, [](const auto& v1, const auto &v2) {
		return std::get<0>(v1) < std::get<0>(v2);
	});
	
	std::cout << "size: " << v1.size() << "\n";
	for(int i=0; i < v1.size(); ++i) {
		std::cout << v1[i] << " " << v2[i] << "\n";
	}
*/

Loop Loop::Offset(float amt) {
	auto offset_curves = std::vector<Curve>{};
	offset_curves.resize(curves_.size());
	for(const auto& curve : curves_) {
		auto idx = curve.which();
		switch(idx) {
			case 0: offset_curves.push_back(::planar::Offset(*curve.template target<LineSegment>(), amt)); break;
			case 1: offset_curves.push_back(::planar::Offset(*curve.template target<Circle>(), amt)); break;
			case 2: offset_curves.push_back(::planar::Offset(*curve.template target<Arc>(), amt)); break;
		}
	}
	return Loop(offset_curves);
}

}
