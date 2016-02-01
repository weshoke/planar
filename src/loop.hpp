#ifndef loop_hpp
#define loop_hpp

#include "primitives.hpp"
#include <vector>

namespace planar {

	class Loop{
	public:
		Loop(const std::vector<Curve> &curves);

		Loop Offset(float amt);
		const std::vector<Curve>& curves() const { return curves_; }

	private:
		std::vector<Curve> curves_;
	};

}

#endif
