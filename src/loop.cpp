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


template <typename T>
void draw(const T& x, ostream& out, size_t position) {
	out << string(position, ' ') << x << "\n";
}


Loop Loop::Offset(float amt) {
	auto tangents = std::vector< std::vector<Vec2d> >{};
	tangents.reserve(curves_.size());
	
	for(const auto& curve : curves_) {
		tangents.push_back(planar::Tangents(curve));
	}
	
	auto sin_theta = std::vector<float>{};
	sin_theta.reserve(tangents.size());
	for(int i=0; i < tangents.size()-1; ++i) {
		const auto &t1 = tangents[i][1];
		const auto &t2 = tangents[i + 1][0];
		sin_theta.push_back((t2 ^ t1)[0]);
	}
	{
		const auto &t1 = tangents.back()[1];
		const auto &t2 = tangents.front()[0];
		sin_theta.push_back((t2 ^ t1)[0]);
	}
	

	auto offset_curves = std::vector<Curve>{};
	offset_curves.reserve(curves_.size());

	
	for(const auto& curve : curves_) {
		offset_curves.push_back(planar::Offset(curve, amt));
	}

	for(int i=curves_.size()-2; i >= 0; --i) {
		const auto &curve = curves_[i];
		const auto &offset_curve1 = offset_curves[i + 1];
		const auto &offset_curve0 = offset_curves[i];
		
		if(sin_theta[i] < 0.f) {
			// needs current offset's endpoint and next offsets startpoint
			auto c = Arc{Circle{Endpoints(curve)[1], amt}, {Endpoints(offset_curve0)[1], Endpoints(offset_curve1)[0]}};
			offset_curves.insert(offset_curves.begin() + i + 1, c);
		}
	}
	{
		const auto &curve = curves_.back();
		const auto &offset_curve1 = offset_curves.front();
		const auto &offset_curve0 = offset_curves.back();
		
		if(sin_theta.back() < 0.f) {
			// needs current offset's endpoint and next offsets startpoint
			auto c = Arc{Circle{Endpoints(curve)[1], amt}, {Endpoints(offset_curve0)[1], Endpoints(offset_curve1)[0]}};
			offset_curves.insert(offset_curves.end() - 1, c);
		}
	}
	
	return Loop(offset_curves);
}

}
