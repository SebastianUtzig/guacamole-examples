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

#include <math.h> 



float SPAWN_TIME = 2.5f;



int main(int argc, char** argv) {

  // initialize guacamole
  gua::init(argc, argv);

  gua::ShadingModelDatabase::load_shading_models_from("data/materials/");
  gua::MaterialDatabase::load_materials_from("data/materials/");

  //setup physics
  gua::physics::Physics physics;
  physics.get_bullet_dynamics_world()->setGravity(btVector3(0,-9.81f,0));

  /*gua::physics::CollisionShapeDatabase::add_shape("box", new gua::physics::BoxShape(2.0f,2.0f,2.0f));
  gua::physics::CollisionShapeDatabase::add_shape("box2", new gua::physics::BoxShape(4.0f,4.0f,4.0f));*/
  //gua::physics::CollisionShapeDatabase::add_shape("box2", new gua::physics::PlaneShape(0.0f,1.0f,0.0f,2.0f));

  auto cs = gua::physics::TriangleMeshShape::FromGeometryFile("data/objects/teapot.obj", true, true, gua::GeometryLoader::OPTIMIZE_GEOMETRY);
  //auto cs = gua::physics::ConvexHullShape::FromGeometryFile("data/objects/teapot.obj",true);
  //cs->set_scaling(gua::math::vec3(0.5,0.5,0.5));
  gua::physics::CollisionShapeDatabase::add_shape("teapot", cs);
  gua::physics::CollisionShapeDatabase::add_shape("sphere", new gua::physics::SphereShape(0.5f));
  gua::physics::CollisionShapeDatabase::add_shape("sphere2", new gua::physics::SphereShape(0.05f));


  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::GeometryLoader loader;

  //gua::PhysicalGeometryLoader phys_loader;
  auto phys_loader = gua::PhysicalGeometryLoader(&physics);


 /* std::shared_ptr<gua::physics::CollisionShapeNode> csn (new gua::physics::CollisionShapeNode("teapot"));
  csn->data.set_shape("teapot");

  auto root_teapot = loader.create_geometry_from_file("teapot_geom", "data/objects/teapot.obj", "Red");
  std::shared_ptr<gua::GeometryNode> geom = std::dynamic_pointer_cast<gua::GeometryNode>(root_teapot);
  //geom->data.set_geometry("");
  auto phys_node = new gua::PhysicalNode(geom,&physics,nullptr,0.0f);//-> mass 0.0 => static
  std::shared_ptr<gua::Node> teapot_geometry(phys_node);*/

  
  //auto phys_node0 = phys_loader.create_physical_objects_from_file("oilrig_geom", "/opt/3d_models/OIL_RIG_GUACAMOLE/oilrig.obj", "Red",true);
  auto phys_node0 = phys_loader.create_physical_objects_from_file("funny_geom", "data/objects/funny.obj", "Red",0.0,nullptr);

  //auto phys_node_shared0 = std::dynamic_pointer_cast<gua::PhysicalNode>(phys_node0);

  //phys_node_shared0->set_mass(1.0);

  //phys_node_shared0->simulate(true);
  
  //phys_node_shared0->set_mass(0.0);
  
  graph.add_node("/",phys_node0);
  
//  phys_node0->scale(0.01, 0.01, 0.01);
  //phys_node0->scale(1.5, 1.5,1.5);
//  phys_node0->rotate(180.0f,0.0f,1.0f,0.0f);
  phys_node0->translate(0.0f, 0.0f, -10.0f);
  phys_node0->simulate(true);


  //static no2
  auto phys_node01 = phys_loader.create_physical_objects_from_file("funny_geom", "data/objects/funny.obj", "Red",0.0,nullptr);
  graph.add_node("/",phys_node01);
  //phys_node0->add_child(phys_node01);
  phys_node01->translate(2.5f, 0.0f, -10.0f);
  phys_node01->simulate(true);

  //static no3
  auto phys_node02 = phys_loader.create_physical_objects_from_file("funny_geom", "data/objects/funny.obj", "Red",0.0,nullptr);
  graph.add_node("/",phys_node02);
  //phys_node01->add_child(phys_node02);
  phys_node02->translate(-2.5f, 0.0f, -10.0f);
  phys_node02->simulate(true);






  /*std::shared_ptr<gua::physics::CollisionShapeNode> csn2 (new gua::physics::CollisionShapeNode("sphere2"));
  csn2->data.set_shape("sphere2");

  auto sphere = loader.create_geometry_from_file("sphere_geom", "data/objects/sphere.obj", "Red");
  std::shared_ptr<gua::GeometryNode> geom2 = std::dynamic_pointer_cast<gua::GeometryNode>(sphere);
  //geom2->data.set_geometry("");
  auto phys_node2 = new gua::PhysicalNode(geom2,&physics,csn2,0.0f);//-> mass 0.0 => static
  std::shared_ptr<gua::Node> ball_geometry(phys_node2);


  graph.add_node("/",ball_geometry);
  sphere->scale(1.0f, 1.0f, 1.0f);
  sphere->translate(0.0f,0.0f, -10.0f);

  phys_node2->simulate(true);*/





  auto screen = graph.add_node<gua::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.6, 0.9));
  screen->translate(0, 0.3f, 1.f);

  auto eye = graph.add_node<gua::TransformNode>("/", "eye");
  eye->translate(0, 0, 2.5);

  unsigned width = 1920;
  unsigned height = 1920 * 9 / 16;

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

  gua::Timer timer2;
  timer2.start();
  float __last_spawn_time = -1;

  double time(0);
  double desired_frame_time(1.0 / 60.0);

  // application loop
  gua::events::MainLoop loop;

  gua::events::Ticker ticker(loop, desired_frame_time);

  ticker.on_tick.connect([&]() {
    double frame_time(timer.get_elapsed());
    time += frame_time;
    timer.reset();

    if (__last_spawn_time == -1 | timer2.get_elapsed() - __last_spawn_time >= SPAWN_TIME){
      __last_spawn_time = timer2.get_elapsed();
      std::cout<<"spawn ball!!!"<<std::endl;

      std::list<std::shared_ptr<gua::PhysicalNode>> group_nodes = std::list<std::shared_ptr<gua::PhysicalNode>>();

/*      std::shared_ptr<gua::physics::CollisionShapeNode> csn2 (new gua::physics::CollisionShapeNode("sphere"));
      csn2->data.set_shape("sphere");*/

      auto phys_node = phys_loader.create_physical_objects_from_file("sphere_geom", "data/objects/cube.obj", "Red",1.0,nullptr);


      graph.add_node("/",phys_node);
      
      phys_node->scale(0.5, 0.5, 0.5);

      //phys_node->rotate(45.0,0,1,0);

      //phys_node->translate(sin(timer2.get_elapsed()) * 0.6f, 7.0f, -10.0f);
      phys_node->translate(0.0f, 7.0f, -10.0f);

      group_nodes.push_back(phys_node); 


      ////////////////////////////////////////////////////////////////

      __last_spawn_time = timer2.get_elapsed();
      std::cout<<"spawn ball!!!"<<std::endl;

/*      std::shared_ptr<gua::physics::CollisionShapeNode> csn3 (new gua::physics::CollisionShapeNode("sphere"));
      csn3->data.set_shape("sphere");*/

      auto phys_node2 = phys_loader.create_physical_objects_from_file("sphere_geom2", "data/objects/cube.obj", "Red",1.0,nullptr);

      //phys_node->get_children()[0]->add_child(phys_node2);
      phys_node->add_child(phys_node2);
      
      phys_node2->translate(5.0f,-5.0f,0.0f);
      group_nodes.push_back(phys_node2);

      /////////////////////////////////////////////////////////////////


      __last_spawn_time = timer2.get_elapsed();
      std::cout<<"spawn ball!!!"<<std::endl;

 /*     std::shared_ptr<gua::physics::CollisionShapeNode> csn4 (new gua::physics::CollisionShapeNode("sphere"));
      csn4->data.set_shape("sphere");*/

      auto phys_node3 = phys_loader.create_physical_objects_from_file("sphere_geom3", "data/objects/cube.obj", "Red",1.0,nullptr);

      //phys_node2->get_children()[0]->add_child(phys_node3);
      phys_node2->add_child(phys_node3);

      group_nodes.push_back(phys_node3);

      phys_node3->translate(-10.0f, 0.0f,0.0f);


      for(auto node : group_nodes){
        node->simulate(true);
      }

    }




    physics.synchronize(true);
    renderer.queue_draw({&graph});
  });
  
  loop.start();

  return 0;
}
