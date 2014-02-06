/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#include <gua/guacamole.hpp> 
#include <gua/physics.hpp>


const std::string geometry("data/objects/cube.obj");
// const std::string geometry("data/objects/cube.obj");

std::vector<std::shared_ptr<gua::TransformNode>> add_lights(gua::SceneGraph& graph,
                                                  int count) {

  std::vector<std::shared_ptr<gua::TransformNode>> lights(count);

  for (int i(0); i < count; ++i) {
    scm::math::vec3 randdir(gua::math::random::get(-1.f, 1.f),
                            gua::math::random::get(-1.f, 1.f),
                            gua::math::random::get(-1.f, 1.f));
    scm::math::normalize(randdir);

    gua::GeometryLoader loader;
    auto sphere_geometry(
      loader.create_geometry_from_file(
      "sphere" + gua::string_utils::to_string(i),
      "data/objects/light_sphere.obj",
      "White"
    ));

    sphere_geometry->scale(0.04, 0.04, 0.04);

    lights[i] = graph.add_node("/", std::make_shared<gua::TransformNode>("light" + gua::string_utils::to_string(i)));
    lights[i]->add_child(sphere_geometry);
    lights[i]->translate(randdir[0], randdir[1], randdir[2]);

    auto light = lights[i]->add_child(std::make_shared<gua::PointLightNode>("light"));
    light->data.set_color(gua::utils::Color3f::random());
    //light->data.set_color(gua::utils::Color3f(0.1,1.0,0.1));
  }

  return lights;
}

void setup_scene(gua::SceneGraph& graph,
                 std::shared_ptr<gua::Node> const& root_monkey,
                 int depth_count,
                 gua::physics::Physics* physics,
                 std::list<gua::PhysicalNode*>& group_nodes) {

  if (depth_count > 0) {
    gua::GeometryLoader loader;

    float offset(2.f);
    std::vector<gua::math::vec3> directions = {
      gua::math::vec3(0, offset, 0),
      gua::math::vec3(0, -offset, 0),
      gua::math::vec3(offset, 0, 0),
      gua::math::vec3(-offset, 0, 0),
      gua::math::vec3(0, 0, offset),
      gua::math::vec3(0, 0, -offset)
    };

    for (auto direction: directions) {
      /*auto monkey_geometry(loader.create_geometry_from_file(
        "monkey",
        geometry,
        "Stones"
      ));*/


      //use physicalgeometrynodes now!!!

      //auto shape = new gua::physics::BoxShape(1,1,1);
      std::shared_ptr<gua::physics::CollisionShapeNode> csn (new gua::physics::CollisionShapeNode("box"));
      csn->data.set_shape("box");

      auto node = loader.create_geometry_from_file("monkey", geometry, "Red");
      std::shared_ptr<gua::GeometryNode> geom = std::dynamic_pointer_cast<gua::GeometryNode>(node);
      auto phys_node = new gua::PhysicalNode(geom,physics,nullptr);
      std::shared_ptr<gua::Node> monkey_geometry(phys_node);


      auto monkey = root_monkey->add_child(monkey_geometry);
      node->scale(0.5, 0.5, 0.5);
      node->translate(direction[0], direction[1], direction[2]-2);
      
      //if(depth_count==3){
        //group_nodes.push_back(phys_node);
        phys_node->make_collidable(true);
      //}

      setup_scene(graph, node, depth_count - 1,physics,group_nodes);
    }
  }
}

int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  gua::ShadingModelDatabase::load_shading_models_from("data/materials/");
  gua::MaterialDatabase::load_materials_from("data/materials/");

  //setup physics
  gua::physics::Physics physics;
  physics.get_bullet_dynamics_world()->setGravity(btVector3(0,-0.81f,0));

  gua::physics::CollisionShapeDatabase::add_shape("box", new gua::physics::BoxShape(0.25f,0.25f,0.25f));
  gua::physics::CollisionShapeDatabase::add_shape("box2", new gua::physics::BoxShape(0.5f,0.5f,0.5f));
  gua::physics::CollisionShapeDatabase::add_shape("box3", new gua::physics::PlaneShape(0.0f,1.0f,0.0f,-0.8f));


  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::GeometryLoader loader;

  /*auto monkey_geometry(loader.create_geometry_from_file(
    "root_ape",
    geometry,
    "Stones"
  ));

  auto root_monkey = graph.add_node("/", monkey_geometry);
  root_monkey->scale(0.5, 0.5, 0.5);*/

  std::shared_ptr<gua::physics::CollisionShapeNode> csn (new gua::physics::CollisionShapeNode("box3"));
  csn->data.set_shape("box3");

  auto root_monkey = loader.create_geometry_from_file("monkey", geometry, "Stones");
  std::shared_ptr<gua::GeometryNode> geom = std::dynamic_pointer_cast<gua::GeometryNode>(root_monkey);
  geom->data.set_geometry("");
  auto phys_node = new gua::PhysicalNode(geom,&physics,csn,0.0f);//-> mass 0.0 => static
  std::shared_ptr<gua::Node> monkey_geometry(phys_node);


  auto monkey = graph.add_node("/",monkey_geometry);
  root_monkey->scale(0.5, 0.5, 0.5);
  root_monkey->scale(1.0, 1.0, 1.0);
  
  phys_node->make_collidable(true);

  // depth    monkey    cube          car
  // 1        14.084      56    3.619.000 Vertices  /      7 draw calls
  // 2        74.444     296   19.129.000 Vertices  /     37 draw calls
  // 3       436.604   1.736  112.189.000 Vertices  /    217 draw calls
  // 4     2.609.564  10.376              Vertices  /  1.297 draw calls
  // 5    15.647.324  62.216              Vertices  /  7.777 draw calls
  // 6    93.873.884 373.256              Vertices  / 46.657 draw calls
  std::list<gua::PhysicalNode*> group_nodes = std::list<gua::PhysicalNode*>();
  setup_scene(graph, root_monkey, 2,&physics,group_nodes);

  for(auto node : group_nodes){
    node->make_collidable(true);
  }

  auto lights = add_lights(graph, 50);

  auto screen = graph.add_node<gua::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.6, 0.9));
  screen->translate(0, 0, 1.f);

  auto eye = graph.add_node<gua::TransformNode>("/", "eye");
  eye->translate(0, 0, 2.5);

  unsigned width = 1500;
  unsigned height = 1500 * 9 / 16;

  auto pipe = new gua::Pipeline();
  pipe->config.set_camera(gua::Camera("/eye", "/eye",
                                      "/screen", "/screen",
                                      "main_scenegraph"));
  pipe->config.set_left_resolution(gua::math::vec2ui(width, height));
  pipe->config.set_enable_fps_display(true);
  pipe->config.set_enable_frustum_culling(true);

  pipe->config.set_enable_ssao(true);
  pipe->config.set_ssao_intensity(2.f);
  pipe->config.set_enable_fxaa(true);
  pipe->config.set_enable_hdr(true);
  pipe->config.set_hdr_key(5.f);
  pipe->config.set_enable_bloom(true);
  pipe->config.set_bloom_radius(10.f);
  pipe->config.set_bloom_threshold(0.8f);
  pipe->config.set_bloom_intensity(0.8f);


  auto window(new gua::Window());
  window->config.set_size(gua::math::vec2ui(width, height));
  window->config.set_left_resolution(gua::math::vec2ui(width, height));
  window->config.set_enable_vsync(true);

  pipe->set_window(window);

  gua::Renderer renderer({
    pipe
  });

  gua::Timer timer;
  timer.start();

  double time(0);
  double desired_frame_time(1.0 / 60.0);

  // application loop
  gua::events::MainLoop loop;

  gua::events::Ticker ticker(loop, desired_frame_time);

  ticker.on_tick.connect([&]() {
    double frame_time(timer.get_elapsed());
    time += frame_time;
    timer.reset();

    /*std::function<void (std::shared_ptr<gua::Node>, int)> rotate;
    rotate = [&](std::shared_ptr<gua::Node> node, int depth) {
      node->rotate(frame_time * (1+depth) * 0.5, 1, 1, 0);
      for (auto child: node->get_children()) {
        rotate(child, ++depth);
      }

    };

    rotate(graph["/root_ape"], 1);

    for (int i = 0; i < lights.size(); ++i) {
      lights[i]->rotate(
          std::sin(time * (i * 0.1 + 0.5)) * frame_time * 2.5, 0, 1, 0);
    }

    graph["/root_ape"]->rotate(15 * frame_time, 0, 1, 0);*/
    //graph["/screen"]->rotate(20*frame_time, 0, 1, 0);

    physics.synchronize(true);
    renderer.queue_draw({&graph});
  });
  
  loop.start();

  return 0;
}
