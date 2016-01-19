#include "lest/lest.hpp"
#include "primitives.hpp"
#include <cmath>

// void TestMarchingCubes(lest::env &lest_env, int size, F f)
// EXPECT(v_old->x == lest::approx(v_new.pos.x));

template<typename T1, typename T2>
void TestIntersection(
	lest::env &lest_env,
	const T1 &elem1,
	const T2 &elem2,
	const std::vector<planar::Point2D> &expected_pts
) {
	auto pts = planar::Intersect(elem1, elem2);
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
	
		
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0., 0.), P2D(0., 1.)},
			{P2D(0., 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(1., 0.), P2D(1., 1.)},
			{P2D(1., 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0.5, 0.), P2D(0.5, 1.)},
			{P2D(0.5, 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0.5, -1.), P2D(0.5, 1.)},
			{P2D(0.5, 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(1.5, -1.), P2D(1.5, 1.)},
			{}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(-.5, -1.), P2D(-.5, 1.)},
			{}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			LineSegment{P2D(0., 1.), P2D(1., 1.)},
			{}
		);
	},
	CASE("Test LineSegment-Circle Intersections") {
		using LineSegment = planar::LineSegment;
		using Circle = planar::Circle;
		using P2D = planar::Point2D;
		
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			Circle{P2D(0., 0.), 1.},
			{P2D(1., 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.9, 0.)},
			Circle{P2D(0., 0.), 1.},
			{}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(-1., 0.), P2D(1., 0.)},
			Circle{P2D(0., 0.), 1.},
			{P2D(-1., 0.), P2D(1., 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(1., -1.), P2D(1., 1.)},
			Circle{P2D(0., 0.), 1.},
			{P2D(1., 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 1.)},
			Circle{P2D(0., 0.), 1.},
			{P2D(std::cos(M_PI_4), std::sin(M_PI_4))}
		);
	},
	CASE("Test LineSegment-Circle Intersections") {
		using LineSegment = planar::LineSegment;
		using Arc = planar::Arc;
		using Circle = planar::Circle;
		using P2D = planar::Point2D;
		
		auto norm = [](const P2D &p) { return p / p.norm(); };
		// Q1
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(1., 1.))}},
			{P2D(1., 0.)}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(1., 1.))}},
			{norm(P2D(1., 0.1))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0., 1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(1., 1.))}},
			{}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(-1., 0.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(1., 1.))}},
			{}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0., -1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(1., 1.))}},
			{}
		);
		
		// Q2
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., 1.))}},
			{norm(P2D(1., 0.1))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.1, 1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., 1.))}},
			{norm(P2D(0.1, 1.))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(-1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., 1.))}},
			{}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.1, -1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., 1.))}},
			{}
		);
		
		// Q3
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{norm(P2D(1., 0.1))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.1, 1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{norm(P2D(0.1, 1.))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(-1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{norm(P2D(-1., 0.1))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.1, -1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{}
		);
		
		// Q4
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{norm(P2D(1., 0.1))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.1, 1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{norm(P2D(0.1, 1.))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(-1., 0.1)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(-1., -1.))}},
			{norm(P2D(-1., 0.1))}
		);
		TestIntersection(
			lest_env,
			LineSegment{P2D(0., 0.), P2D(0.1, -1.)},
			Arc{Circle{P2D(0., 0.), 1.}, LineSegment{P2D(1., 0.), norm(P2D(1., -1.))}},
			{norm(P2D(0.1, -1.))}
		);
	}
};
// clang-format on

int main(int argc, char *argv[])
{
	return lest::run(specification, argc, argv);
}
