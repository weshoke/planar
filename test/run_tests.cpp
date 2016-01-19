#include "lest/lest.hpp"
#include "primitives.hpp"

// void TestMarchingCubes(lest::env &lest_env, int size, F f)
// EXPECT(v_old->x == lest::approx(v_new.pos.x));

void TestLineSegmentLineSegmentIntersection(
	lest::env &lest_env,
	const planar::LineSegment &LS1,
	const planar::LineSegment &LS2,
	const std::vector<planar::Point2D> &expected_pts
) {
	auto pts = planar::Intersect(LS1, LS2);
	//std::cout << "SIZES: " << pts.size() << " " << expected_pts.size() << "\n";
	EXPECT(pts.size() == expected_pts.size());
	for(size_t i=0; i < pts.size(); ++i) {
		EXPECT(pts[i][0] == lest::approx(expected_pts[i][0]));
		EXPECT(pts[i][1] == lest::approx(expected_pts[i][1]));
	}
}

// clang-format off
const lest::test specification[] = {
	CASE("Test LineSegment-LineSegment Intersections") {
		using LineSegment = planar::LineSegment;
		using P2D = planar::Point2D;
	
		
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0., 0.), P2D(0., 1.)},
			{P2D(0., 0.)}
		);
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(1., 0.), P2D(1., 1.)},
			{P2D(1., 0.)}
		);
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0.5, 0.), P2D(0.5, 1.)},
			{P2D(0.5, 0.)}
		);
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0.5, -1.), P2D(0.5, 1.)},
			{P2D(0.5, 0.)}
		);
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(1.5, -1.), P2D(1.5, 1.)},
			{}
		);
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(-.5, -1.), P2D(-.5, 1.)},
			{}
		);
		TestLineSegmentLineSegmentIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0., 1.), P2D(1., 1.)},
			{}
		);
	}
};
// clang-format on

int main(int argc, char *argv[])
{
	return lest::run(specification, argc, argv);
}
