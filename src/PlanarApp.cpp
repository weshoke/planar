#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Log.h"
#include "cinder/CameraUi.h"
#include "vsr/space/vsr_cga2D.h"
#include "primitives.hpp"
#include "loop.hpp"
#include <cmath>

using namespace ci;
using namespace ci::app;
using namespace std;

class PlanarApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void mouseDrag( MouseEvent event ) override;
	void update() override;
	void resize() override;
	void draw() override;
	
	CameraPersp camera;
	CameraUi camUi;
	std::vector<gl::BatchRef> batches;
};

gl::GlslProgRef LoadShader(const std::string& name, bool has_geometry_shader=true) {
	gl::GlslProgRef shader;
	try {
		auto format = gl::GlslProg::Format()
		.vertex(loadAsset(name + ".vert"))
		.fragment(loadAsset(name + ".frag"));
		
		if(has_geometry_shader) {
			format = format.geometry(loadAsset(name + ".geom"));
		}
		
		shader = gl::GlslProg::create(format);
	}
	catch(Exception &exc) {
		CI_LOG_E("error loading " << name << " shader: " << exc.what());
	}
	return shader;
}

gl::BatchRef ToBatch(const planar::LineSegment &segment, glm::vec4 color = glm::vec4(0.f, 0.f, 0.f, 1.f)) {
	auto position = std::vector<glm::vec3>{
		glm::vec3(segment.pts[0][0], segment.pts[0][1], 0.f),
		glm::vec3(segment.pts[1][0], segment.pts[1][1], 0.f)
	};
	auto mesh = gl::VboMesh::create(position.size(), GL_LINES, {gl::VboMesh::Layout().attrib(geom::POSITION, 3)});
	mesh->bufferAttrib(geom::POSITION, position);
	
	auto shader = LoadShader("pass", false);
	shader->uniform("color", color);
	return gl::Batch::create(mesh, shader);
}


gl::BatchRef ToBatch(const planar::Circle &circle, glm::vec4 color = glm::vec4(0.f, 0.f, 0.f, 1.f)) {
	auto position = std::vector<glm::vec3>{};
	auto size = 40;
	position.reserve(size);
	
	auto dtheta = 2.f * M_PI / float(size + 1);
	for(int i=0; i < size; ++i) {
		auto x = circle.radius * std::cos(dtheta * float(i)) + circle.center[0];
		auto y = circle.radius * std::sin(dtheta * float(i)) + circle.center[1];
		position.push_back(glm::vec3(x, y, 0.f));
	}
	
	auto mesh = gl::VboMesh::create(position.size(), GL_LINE_LOOP, {gl::VboMesh::Layout().attrib(geom::POSITION, 3)});
	mesh->bufferAttrib(geom::POSITION, position);
	
	auto shader = LoadShader("pass", false);
	shader->uniform("color", color);
	return gl::Batch::create(mesh, shader);
}

gl::BatchRef ToBatch(const planar::Arc &arc, glm::vec4 color = glm::vec4(0.f, 0.f, 0.f, 1.f)) {
	auto dir0 = arc.endpoints.pts[0] - arc.circle.center;
	auto dir1 = arc.endpoints.pts[1] - arc.circle.center;
	auto ip = (dir0 <= dir1)[0];
	auto op = (dir0 ^ dir1)[0];
	//std::cout << "ip: " << ip << "\n";
	//std::cout << "op: " << op << "\n";
	
	// TODO: reorganize this to not calc asin if not needed
	auto theta_range = std::asin(op);
	//std::cout << "theta_range: " << theta_range << " " << (theta_range * 180.f / M_PI) << "\n";
	if(ip < 0.f) {
		theta_range = M_PI - theta_range;
	}
	else if(op < 0.f) {
		theta_range += 2.f * M_PI;
	}
	
	if(arc.circle.radius < 0.f) {
		theta_range = theta_range - 2.f * M_PI;
	}
	
	
	//std::cout << "theta_range: " << theta_range << "\n";
	
	auto theta_start = std::atan2(dir0[1], dir0[0]);
	auto n = int(std::abs(theta_range) / (M_PI / 40.f)) + 1;
	auto dtheta = theta_range / float(n);
	
	auto radius = std::abs(arc.circle.radius);
	auto position = std::vector<glm::vec3>{};
	position.reserve(n);
	position.push_back(glm::vec3(arc.endpoints.pts[0][0], arc.endpoints.pts[0][1], 0.f));
	for(int i=1; i < n; ++i) {
		auto x = radius * std::cos(theta_start + dtheta * float(i)) + arc.circle.center[0];
		auto y = radius * std::sin(theta_start + dtheta * float(i)) + arc.circle.center[1];
		position.push_back(glm::vec3(x, y, 0.f));
	}
	position.push_back(glm::vec3(arc.endpoints.pts[1][0], arc.endpoints.pts[1][1], 0.f));
	
	
	auto mesh = gl::VboMesh::create(position.size(), GL_LINE_STRIP, {gl::VboMesh::Layout().attrib(geom::POSITION, 3)});
	mesh->bufferAttrib(geom::POSITION, position);
	
	auto shader = LoadShader("pass", false);
	shader->uniform("color", color);
	return gl::Batch::create(mesh, shader);
}

void PlanarApp::setup()
{
	// 1: scalar
	// 2: e1, e2, no, ni
	// 3: e12, e1no, e1ni, e2no, e2ni, noni
	// 4: e12no, e12ni, e1noni, e2noni
	// 5: e12noni
	camera.lookAt(normalize(vec3(0, 0, 1)) * 5.0f, vec3(0.f, 0.f, 0.f));
	camUi = CameraUi(&camera);
	ci::app::getWindow()->setTitle("Planar");

	// Pnt
	// e1 e2 no ni
	/*
	e1	000001	1
	e2	000010	2
	e3	000100	4
	e4	001000	8
	*/
//	auto pt1 = vsr::cga2D::point(vsr::cga2D::Vec(0., 0.));
//	auto pt2 = vsr::cga2D::point(vsr::cga2D::Vec(1., 0.));
//	std::cout << pt1 << "\n";
//	std::cout << pt2 << "\n";
//	std::cout << vsr::nga::Round::radius(pt1) << " " << vsr::nga::Round::size(pt1, false) << "\n";
//	
//	std::cout << "\n";
	// Dual circle
	/*
	e1	000001	1
	e2	000010	2
	e3	000100	4
	e4	001000	8
	*/
//	auto c1 = vsr::cga2D::circle(0., 0., 1.);
//	auto c2 = vsr::cga2D::circle(2., 0., 1.);
//	std::cout << c1 << "\n";
//	std::cout << c2 << "\n";
	
	// Circle
	/*
	e123	000111	7
	e124	001011	11
	e134	001101	13
	e234	001110	14
	*/
	
	// Par
	/*
	e12	000011	3
	e13	000101	5
	e23	000110	6
	e14	001001	9
	e24	001010	10
	e34	001100	12
	*/
	
	// Lin
	/*
	e124	001011	11
	e134	001101	13
	e234	001110	14
	*/
	
	// DLL
	/*
	e1	000001	1
	e2	000010	2
	e4	001000	8
	*/
	
	//std::cout << c1 ^ vsr::cga2D::
	/*
	printf("vector\n");
	vsr::cga2D::Vec::basis::print();
	vsr::cga2D::Pnt::basis::print();
	vsr::cga2D::Dls::basis::print();
	vsr::cga2D::Cir::basis::print();
	vsr::cga2D::Par::basis::print();
	vsr::cga2D::Lin::basis::print();
	vsr::cga2D::Dll::basis::print();
	*/
	
	/*
	// Point pair
	auto pp = (c1 ^ c2).dual();
	std::cout << pp << "\n";
	std::cout << vsr::nga::Round::radius(pp) << " " << vsr::nga::Round::size(pp, false) << "\n";
	
	auto pts = vsr::nga::Round::split(pp);
	std::cout << "\n";
	std::cout << pts[0] << "\n";
	std::cout << pts[1] << "\n";
	
	
	std::cout << "**********\n";
	
	auto pp1 = pt1 ^ pt2;
	std::cout << pp1 << "\n";
	std::cout << vsr::nga::Round::radius(pp1) << " " << vsr::nga::Round::size(pp1, false) << "\n";
	
	auto pt3 = vsr::cga2D::point(vsr::cga2D::Vec(0.5, 0.));
	std::cout << "X: " << (pt3 ^ pp1.dual()) << "\n";
	std::cout << "X: " << (c2 <= pt3) << " " << sqrt(((c2 <= pt3)[0]) * -2.f) << "\n";
	// Test carrier and surround
	// If in surround (pp -> circle)
	// Test if on line
	
	// 2 segments
	// find intersection of 2 line carriers
	// see if that point is on ther interval
	
	auto sur1 = vsr::nga::Round::surround(pp1);
	std::cout << sur1 << "\n";
	std::cout << vsr::nga::Round::radius(sur1) << " " << vsr::nga::Round::size(sur1, true) << "\n";
	decltype(sur1)::basis::print();
	
	auto car1 = vsr::nga::Round::carrier(pp1);
	std::cout << car1 << "\n";
	decltype(car1)::basis::print();
	*/
	//std::cout << vsr::cga2D::circle(0.5, 0., 1.) << "\n";
	
	/*
	auto LS1 = LineSegment{
		{vsr::cga2D::Vec(0., 0.), vsr::cga2D::Vec(1., 0.)}
	};
	auto LS2 = LineSegment{
		{vsr::cga2D::Vec(1.5, -1.), vsr::cga2D::Vec(1.5, 3.)}
		//{vsr::cga2D::Vec(0., -1.), vsr::cga2D::Vec(1., -1.000001)}
	};
	Intersect(LS1, LS2);
	auto C = Circle{
		vsr::cga2D::Vec(0.5, 0.),
		0.25
	};
	auto v1 = Intersect(C, LS1);
	
	std::cout << "\n\n";
	
	auto C2 = Circle{
		vsr::cga2D::Vec(0.5, 1.),
		0.25
	};
	auto v2 = Intersect(C2, LS1);
	
	
	std::cout << "\n\n";
	
	auto C3 = Circle{
		vsr::cga2D::Vec(0.5, 0.25),
		0.25
	};
	auto v3 = Intersect(C3, LS1);
	std::cout << v1.size() << "\n";
	std::cout << v2.size() << "\n";
	std::cout << v3.size() << "\n";
	*/
	
	auto LS1 = planar::LineSegment{
		{vsr::cga2D::Vec(0., 0.), vsr::cga2D::Vec(-1., 2.)}
	};
	auto C2 = planar::Circle{
		vsr::cga2D::Vec(0.5, 1.),
		0.25
	};
	
	auto v2 = vsr::cga2D::Vec(-1., -1.);
	v2 = v2 / v2.norm();
	auto arc1 = planar::Arc{
		planar::Circle{vsr::cga2D::Vec(0.f, 0.f), -1.f},
		planar::LineSegment{{vsr::cga2D::Vec(1., 0.), v2}}
	};
	
	
	
	batches.push_back(ToBatch(LS1));
	//batches.push_back(ToBatch(C2));
	batches.push_back(ToBatch(arc1));
	
	//auto pts = Intersect(LS1, arc1);
	
	/*
	auto v1 = vsr::cga2D::Vec(1., 0.);
	auto v2 = vsr::cga2D::Vec(-1., -0.1);
	std::cout << (v1 <= v2) << "\n";
	std::cout << (v1 ^ v2) << "\n";
	std::cout << (v2 <= v1) << "\n";
	std::cout << (v2 ^ v1) << "\n";
	decltype((v2 ^ v1))::basis::print();
	*/
	
	
	
	/*
	auto loop = Loop({LS1, C2});
	auto loop2 = loop.offset(1.);
	
	for(auto c : loop.curves()) {
		if(c.target_type() == typeid(LineSegment)) {
			auto e = *c.template target<LineSegment>();
			std::cout << e.pts[0] << " " << e.pts[1] << "\n";
		}
		else {
			auto e = *c.template target<Circle>();
			std::cout << e.center << " " << e.radius << "\n";
		}
	}
	
	for(auto c : loop2.curves()) {
		if(c.target_type() == typeid(LineSegment)) {
			auto e = *c.template target<LineSegment>();
			std::cout << e.pts[0] << " " << e.pts[1] << "\n";
		}
		else {
			auto e = *c.template target<Circle>();
			std::cout << e.center << " " << e.radius << "\n";
		}
	}
	*/
	
	/*
	auto pt1 = vsr::cga2D::point(vsr::cga2D::Vec(0., 0.));
	auto pt2 = vsr::cga2D::point(vsr::cga2D::Vec(1., 0.));
	// Par: radius = 0.5
	auto pp = pt1 ^  pt2;
	// Dual Circle aka Point: radius 0.5, center [0.5, 0]
	auto sur = vsr::nga::Round::surround(pp);
	// Line: direction [1, 0]
	//auto car = vsr::nga::Round::carrier(pp);
	
	std::cout << vsr::nga::Round::center(sur) << "\n";
	std::cout << vsr::nga::Round::center(pt1) << "\n";
	std::cout << vsr::nga::Round::center(pt2) << "\n";
	
	auto pt = vsr::cga2D::point(vsr::cga2D::Vec(0.5, 2.));
	auto c = vsr::cga2D::circle(2., 0., 1.);
	auto cs = vsr::nga::Round::size(c, true);
	auto x = pt1 <= c;
	
	//decltype(x)::basis::print();
	std::cout << "x:" << x << " " << std::sqrt(x[0] * -2. + cs) << "\n";
	std::cout << "Cs: " << cs << "\n";
	
	auto ppx = pt1 ^ c;
	std::cout << vsr::nga::Round::center(ppx) << "\n";
	std::cout << vsr::nga::Round::radius(ppx) << " " << vsr::nga::Round::size(ppx, false) << "\n";
	*/
	
	//std::cout << pp << "\n";
	//std::cout << 0.5 << " = " << vsr::nga::Round::radius(pp) << "\n";
	//std::cout << sur << "\n";
	//std::cout << vsr::nga::Round::radius(sur) << "\n";
	
	//std::cout << car << "\n";
	//std::cout << vsr::nga::Flat::direction(car) << "\n";
	//decltype(car)::basis::print();
}

void PlanarApp::mouseDown(MouseEvent event) {
	camUi.mouseDown(event);
}

void PlanarApp::mouseDrag(MouseEvent event) {
	camUi.mouseDrag(event);
}

void PlanarApp::resize() {
	camera.setAspectRatio(getWindowAspectRatio());
}

void PlanarApp::update() {
}

void PlanarApp::draw() {
	gl::clear(Color(0.93, 0.93, 0.93));
	gl::setMatrices(camera);
	for(const auto& batch : batches) {
		batch->draw();
	}
}

CINDER_APP( PlanarApp, RendererGl )
