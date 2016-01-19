#ifndef loop_hpp
#define loop_hpp

#include "primitives.hpp"
#include <vector>

namespace planar {

	class Loop{
	public:
		static Loop Create(const std::vector<Curve> &curves);

		Loop Offset(float amt);
		const std::vector<uint32_t>& elements() const { return elements_; }

	private:
		Loop();

		std::vector<uint32_t> elements_;
		// TODO: is there a way to do this automatically with template meta-programming?
		std::vector<LineSegment> line_segments_;
		std::vector<Circle> circles_;
		std::vector<Arc> arcs_;
	};

}

#endif
