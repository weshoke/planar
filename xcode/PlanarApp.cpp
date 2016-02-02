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
	auto radius = std::abs(arc.circle.radius);
	auto dir0 = (arc.endpoints.pts[0] - arc.circle.center) / radius;
	auto dir1 = (arc.endpoints.pts[1] - arc.circle.center) / radius;
	auto ip = (dir0 <= dir1)[0];
	auto op = (dir0 ^ dir1)[0];
//	std::cout << "ip: " << ip << "\n";
//	std::cout << "op: " << op << "\n";
	
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
	using namespace planar;

	camera.lookAt(normalize(vec3(0, 0, 1)) * 5.0f, vec3(0.f, 0.f, 0.f));
	camUi = CameraUi(&camera);
	ci::app::getWindow()->setTitle("Planar");

	auto ToBatches = [&](const Loop &loop, glm::vec4 color) {
		for(const auto &curve : loop.curves()) {
			switch(TargetType(curve)) {
				case Curve::CurveType::LineSegment: batches.push_back(ToBatch(*(LineSegment*)Target(curve), color)); break;
				case Curve::CurveType::Circle: batches.push_back(ToBatch(*(Circle*)Target(curve), color)); break;
				case Curve::CurveType::Arc: batches.push_back(ToBatch(*(Arc*)Target(curve), color)); break;
			}
		}
	};

	
	auto loop1 = Loop(std::vector<Curve>{
		LineSegment{Point2d(1., 1.), Point2d(-1., 1)},
		LineSegment{Point2d(-1., 1.), Point2d(-1., -1)},
		LineSegment{Point2d(-1., -1.), Point2d(1., -1)},
		LineSegment{Point2d(1., -1.), Point2d(1., 1)}
	});
	auto loop2 = loop1.Offset(0.2);
	
	ToBatches(loop1, vec4(0., 0., 0., 1.));
	ToBatches(loop2, vec4(1., 0.2, 0.2, 1.));
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
