#include <cstdint>
#include <iostream>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include "collider_mesh.h"
#include "mesh.h"
#include "glcorearb.h"
#include "math.h"
#include "shader.h"
#include "input_layer.h"
#include "render_cubes.h"
#include "mesh_renderer.h"
#include "camera.h"
#include "types.h"

#include "collision_detection.h"

#include "physics.h"

// Dear IMGUI
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"
#include "vector.h"

#define WIN_WIDTH	740
#define WIN_HEIGHT	680
#define WIN_NAME	"Test"

#include "kv_storage.h"

#include "sdf.h"

void temp_error_callback(int error_code, const char* descr) {
	std::cout << "GLFW Error: " << error_code << " " << descr << std::endl;
}

// INPUT MOUSE CALLBACk
void key_callback(GLFWwindow *wind, int key, int scancode, int action, int mods) {
	// ESC to close the game
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(wind, GL_TRUE);
	}
}

sVector3 rotate_arround(const sVector3 pos, 
                        const sVector3 center, 
                        const float angle) {
  float x = pos.x - center.x, z = pos.z - center.z;
  float s = sin(angle), c = cos(angle);

  float nx = pos.x * c - pos.z * s;
  float nz = pos.x * s + pos.z * c;

  return sVector3{nx + center.x, pos.y, nz + center.z};
}

#include "sat.h"
void test_loop(GLFWwindow *window) {
	glfwMakeContextCurrent(window);
  char names[4][5] = {
    "WHIT",
    "BLAK",
    "GREN",
    "BLU"
  };
  sTransform   transforms[6] = {};

  sMesh sphere, cube;
  sphere.load_OBJ_mesh("resources/sphere.obj");
  cube.load_OBJ_mesh("resources/cube_t.obj");

  // Object 1: Static cube
  transforms[0].position = {0.0f, 0.0f, 0.0f};
  transforms[0].scale = {1.0f, 10.f, 10.0f};
  transforms[0].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});

  transforms[1].position = {1.206f, 0.0f, 0.0f};
  transforms[1].scale = {1.0f, 1.f, 1.0f};
  //transforms[1].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  transforms[1].set_rotation({0.0f, 0.534f, 0.607f, -0.5880f});


  sMeshRenderer sphere_renderer, cube_renderer;
  cube_renderer.create_from_mesh(&cube);
  sphere_renderer.create_from_mesh(&sphere);

  //cube.clean();
  sphere.clean();

  sVector4 colors[6] = {};
  colors[0] = {1.0f, 1.0f, 1.0f, 0.50f};
  colors[1] = {1.0f, 0.0f, 0.0f, 0.50f};
  colors[2] = {0.0f, 1.0f, 0.0f, 0.50f};
  colors[3] = {0.0f, 0.0f, 1.0f, 0.50f};
  colors[4] = {0.0f, 1.0f, 1.0f, 0.50f};


  float prev_frame_time = glfwGetTime();
  sCamera camera = {};
  float camera_rot = 0.0f;

  //camera.position = {-5.0f, 1.5f, 5.0f};
  camera.position = {5.0f, 1.5f, 5.0f};

  // Frame counter
  int frames = 0;
  double start_time, fps;
  double delta_time = 0.01;
  double accumulator = 0.0;

  // Diagnostics
  float physics_ticks_per_frame[6] = {0.0f, 0.0f, 0.0f, 0.0f,};

  start_time = glfwGetTime();


  sVector3 sphere_pos = {1.2f, 2.0f, 0.0f};

  while(!glfwWindowShouldClose(window)) {
    // Draw loop
    int width, heigth;
    double temp_mouse_x, temp_mouse_y;

    glfwPollEvents();

    glfwGetFramebufferSize(window, &width, &heigth);
    // Set to OpenGL viewport size anc coordinates
    glViewport(0,0, width, heigth);

    sMat44 proj_mat = {};

    // OpenGL stuff
    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();


    camera_rot = 1.10f;
    camera.position = rotate_arround(camera.position,
                                     sVector3{0.0f, 0.0f, 0.0f},
                                     to_radians(camera_rot));

    camera.look_at(sVector3{0.0f, 0.0f, 0.0f});
    camera.get_perspective_viewprojection_matrix(90.0f,
                                                1000.0f,
                                                0.001f,
                                                (float)width / (float)heigth,
                                                &proj_mat);

    double curr_frame_time = glfwGetTime();
    double elapsed_time = curr_frame_time - prev_frame_time;
    prev_frame_time = curr_frame_time;

    // Simulation Update ====


    sMat44 cube_models[35] = {}, sphere_models[15] = {};
    sVector4 cube_colors[35] = {}, sphere_colors[15] = {};
    int cube_size = 0, sphere_size = 0;

    // Rendering ====
    cube_models[0].set_identity();
    transforms[0].get_model(&cube_models[0]);
    cube_models[1].set_identity();
    transforms[1].rotation = transforms[1].rotation.normalize();
    transforms[1].get_model(&cube_models[1]);

    cube_colors[0] = {1,1,1,1};

    ImGui::Begin("Test");
    ImGui::SliderFloat3("Cube2 pos", transforms[1].position.raw_values, -10.0f, 10.0f);
    ImGui::SliderFloat4("Cube2 rot", transforms[1].rotation.raw_values, -10.0f, 10.0f);
    ImGui::SliderFloat3("Sphere pos", sphere_pos.raw_values, -10.0f, 10.0f);


    sColliderMesh col_cube1 = {}, col_cube2 = {};

    col_cube1.init_cuboid(transforms[0]);
    col_cube2.init_cuboid(transforms[1]);
    //col_cube2.load_collider_mesh(cube);
    //col_cube2.apply_transform(transforms[1]);

    sCollisionManifold manifold = {};

    uint32_t cube_num = 2;

    /*for(int j = 0 ; j < col_cube1.face_count; j++){
      cube_colors[cube_num] = {0.0f, 1.f, 0.0f, 0.0f};
      cube_models[cube_num].set_identity();
      cube_models[cube_num].set_scale({0.05f, 0.05f, 0.05f});
      cube_models[cube_num++].add_position(col_cube1.plane_origin[j]);
    }*/

    for(int j = 99 ; j < col_cube2.vertices_count; j++){
      cube_colors[cube_num] = {0.0f, 1.f, 0.0f, 0.0f};
      cube_models[cube_num].set_identity();
      cube_models[cube_num].set_scale({0.05f, 0.05f, 0.05f});
      cube_models[cube_num++].add_position(col_cube2.vertices[j]);
    }

    sphere_models[0].set_identity();
    sphere_models[0].set_position(sphere_pos);
    sphere_renderer.render(sphere_models, colors, 1, proj_mat, true);

    /*
    if (SAT::SAT_collision_test(col_cube1,
                                col_cube2,
                                &manifold)) {
      ImGui::Text("Collision cube %i points", manifold.contanct_points_count);
      for(uint32_t i = 0; i < manifold.contanct_points_count; i++) {
        cube_colors[cube_num] = {1.0f, 0.f, 0.0f, 0.0f};
        cube_models[cube_num].set_identity();
        cube_models[cube_num].set_scale({0.05f, 0.05f, 0.05f});
        cube_models[cube_num++].add_position(manifold.contact_points[i]);
        ImGui::Text("Point %f %f %f, depth: %f", manifold.contact_points[i].x, manifold.contact_points[i].y, manifold.contact_points[i].z, manifold.contact_depth[i]);

      }

    } else {
      ImGui::Text("No cube collision");
    }

    if (SAT::SAT_sphere_cube_collision(sphere_pos,
                                       1.0f,
                                       transforms[0],

                                       col_cube1,

                                       &manifold)) {
      ImGui::Text("Collision sphere %i points", manifold.contanct_points_count);
      for(uint32_t i = 0; i < manifold.contanct_points_count; i++) {
        cube_colors[cube_num] = {1.0f, 0.f, 0.0f, 0.0f};
        cube_models[cube_num].set_identity();
        cube_models[cube_num].set_scale({0.05f, 0.05f, 0.05f});
        cube_models[cube_num++].add_position(manifold.contact_points[i]);
        ImGui::Text("Point %f %f %f, depth: %f", manifold.contact_points[i].x, manifold.contact_points[i].y, manifold.contact_points[i].z, manifold.contact_depth[i]);

      }

    } else {
      ImGui::Text("No sphere collision");
    }
    */

    ImGui::End();


    col_cube1.clean();
    col_cube2.clean();
    //cube_renderer.render(cube_models+1, cube_colors+1, cube_num-1, proj_mat, true);
    cube_renderer.render(cube_models, cube_colors, cube_num, proj_mat, true);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }
}

uint32_t add_sphere(const sVector3 &pos, const float radius, const uint32_t last_index, sPhysWorld &phys_world) {

  phys_world.transforms[last_index].position = pos;
  phys_world.transforms[last_index].scale = {radius, radius, radius};
  phys_world.transforms[last_index].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  phys_world.restitution[last_index] = 0.1f;
  phys_world.friction[last_index] = 0.5f;
  phys_world.mass[last_index] = 50.0f;
  phys_world.shape[last_index] = SPHERE_COLLIDER;
  phys_world.is_static[last_index] = false;
  phys_world.enabled[last_index] = true;

  return last_index+1;
}

uint32_t add_cube(const sVector3 &pos, const sVector3 &scale, const uint32_t last_index, sPhysWorld &phys_world) {

  phys_world.transforms[last_index].position = pos;
  phys_world.transforms[last_index].scale = scale;
  phys_world.transforms[last_index].set_rotation({0.80f, 0.20f, 0.00f, 0.0f});
  //phys_world.transforms[last_index].set_rotation({1.0f, 0.0f, 0.00f, 0.0f});
  phys_world.restitution[last_index] = 0.2f;
  phys_world.friction[last_index] = 0.5f;
  phys_world.mass[last_index] = 10.0f;
  phys_world.shape[last_index] = CUBE_COLLIDER;
  phys_world.is_static[last_index] = false;
  phys_world.enabled[last_index] = true;

  return last_index+1;
}


void draw_loop(GLFWwindow *window) {
	glfwMakeContextCurrent(window);
  char names[4][5] = {
    "WHIT",
    "BLAK",
    "GREN",
    "BLU"
  };
  sTransform   transforms[10] = {};

  sMesh sphere, cube;
  sphere.load_OBJ_mesh("resources/sphere.obj");
  cube.load_OBJ_mesh("resources/cube_t.obj");

  sMeshRenderer sphere_renderer, cube_renderer;
  sphere_renderer.create_from_mesh(&sphere);
  cube_renderer.create_from_mesh(&cube);

  sphere.clean();
  cube.clean();

  sPhysWorld phys_instance;

  phys_instance.init();

  phys_instance.set_default_values();

  uint32_t static_cube = phys_instance.add_cube_collider({0.0f, 0.5f, 0.0f},
                                                         {13.0f, 1.0f, 13.0f},
                                                         0.0f,
                                                         20.0f,
                                                         true);
  /*uint32_t dynamic_cube = phys_instance.add_cube_collider({0.0f, 2.5f, 0.0f},
                                                         {1.0f, 1.0f, 1.0f},
                                                         20.0f,
                                                         false);*/
  /*uint32_t dynamic_cube1 = phys_instance.add_cube_collider({-0.4f, 6.5f, 0.0f},
                                                           {0.50f, 1.0f, 0.90f},
                                                           30.0f,
                                                           0.2f,
                                                           false);*/
  uint32_t dynamic_sphere = phys_instance.add_sphere_collider({0.0f, 2.15, 0.0f},
                                                              0.50f,
                                                              10.0f,
                                                              1.0f,
                                                              false);
  uint32_t dynamic_sphere2 = phys_instance.add_sphere_collider({0.250f, 3.95, 0.0f},
                                                              0.50f,
                                                              5.0f,
                                                              1.0f,
                                                              false);

  //phys_instance.transforms[static_cube].set_rotation({0.9540f, 0.3f, 0.0f, 0.0f});

  std::cout << dynamic_sphere << std::endl;
  sVector4 colors[6] = {};
  colors[0] = {1.0f, 1.0f, 1.0f, 0.50f};
  colors[1] = {0.0f, 0.0f, 0.0f, 0.50f};
  colors[2] = {0.0f, 1.0f, 0.0f, 0.50f};
  colors[3] = {0.0f, 0.0f, 1.0f, 0.50f};
  colors[4] = {0.0f, 0.0f, 1.0f, 1.00f};
  colors[5] = {0.0f, 0.0f, 1.0f, 1.00f};

  sVector4 colors_sp[6] = {};
  colors_sp[0] = {0.0f, 1.0f, 0.0f, 0.50f};
  colors_sp[1] = {0.0f, 0.0f, 1.0f, 0.50f};
  colors_sp[2] = {0.0f, 0.0f, 1.0f, 1.00f};
  colors_sp[3] = {0.0f, 0.0f, 1.0f, 1.00f};


  float prev_frame_time = glfwGetTime();
  sCamera camera = {};
  float camera_rot = 0.0f, camera_height = 3.0f;

  //camera.position = {-5.0f, 1.5f, 5.0f};
  camera.position = {20.0f, 10.5f, 20.0f};

  // Frame counter
  int frames = 0;
  double start_time, fps;
  double delta_time = 0.009;
  double accumulator = 0.0;


  start_time = glfwGetTime();
  camera_rot = 72.10f;
  bool stopped = true;
  while(!glfwWindowShouldClose(window)) {

    /*sTransform transf;
    transf.set_rotation({0.954f, 0.30f, 0.0f, 0.0f});
    sVector3 norm = {0.0f, 1.0f, 0.0f};

    std::cout << norm.x << " " << norm.y <<  " " << norm.z << std::endl;

    norm = transf.apply_rotation(norm);

    std::cout << norm.x << " " << norm.y <<  " " << norm.z << std::endl;*/

    // Draw loop
    int width, heigth;
    double temp_mouse_x, temp_mouse_y;

    glfwPollEvents();
    glfwGetFramebufferSize(window, &width, &heigth);
    // Set to OpenGL viewport size anc coordinates
    glViewport(0,0, width, heigth);

    sMat44 proj_mat = {};

    // OpenGL stuff
    glEnable(GL_DEPTH_TEST);  
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    camera.position = rotate_arround({5.0f, camera_height, 5.0f},
                                     sVector3{0.0f, 0.0f, 0.0f}, 
                                     to_radians(camera_rot));
  
    camera.look_at(sVector3{0.0f, 0.0f, 0.0f});
    camera.get_perspective_viewprojection_matrix(90.0f, 
                                                1000.0f,
                                                0.001f,
                                                (float)width / (float)heigth,
                                                &proj_mat);

    double curr_frame_time = glfwGetTime();
    double elapsed_time = curr_frame_time - prev_frame_time;
    prev_frame_time = curr_frame_time;

    // Simulation Update ====

    ImGui::Begin("Physics");
    ImGui::Text("FPS %f elapsed time %f", 1.0f / ( elapsed_time), elapsed_time);

    int space_state = glfwGetKey(window, GLFW_KEY_SPACE);
    int left_state = glfwGetKey(window, GLFW_KEY_RIGHT);
    if (!stopped) {
      // If the simulation is not stopped, add the Stop button to the GUI,
      // and continue the simulation as normal
      if (ImGui::Button("Stop") || space_state == GLFW_PRESS) {
        stopped = true;
      }

      // Simulate with a fixed timestep
      int num_of_physics_steps = 0;
      accumulator += elapsed_time;
      while(accumulator >= delta_time) {
        phys_instance.step(delta_time);
        accumulator -= delta_time;
        num_of_physics_steps++;
      }
      ImGui::Text("Num of steps: %d", num_of_physics_steps);
    } else {
      // If the simulation is stopped, add a continue button, and a step button,
      // to add a delta timestep to the simulation
      if (ImGui::Button("Continue")) {
        stopped = false;
      }
      if (ImGui::Button("Step") || left_state == GLFW_PRESS) {
        phys_instance.step(delta_time);

        //sVector3 cube_pos = phys_instance.transforms[static_cube].position;
        //sVector3 sphere_center = phys_instance.transforms[dynamic_sphere].position;
        //uint32_t face = phys_instance.collider_meshes[static_cube].get_support_face(sphere_center.subs(cube_pos).invert());

        //sVector3 *vert = phys_instance.collider_meshes[static_cube].get_face(face);

        //std::cout << "SDF: " << SDF::quad(sphere_center, vert[0], vert[1], vert[2], vert[4], phys_instance.collider_meshes[static_cube].normals[face]) << std::endl;// - phys_instance.transforms[dynamic_sphere].scale.x << std::endl;
      }
    }
    phys_instance.debug_speeds();
    ImGui::End();

    ImGui::Begin("Overall");
    ImGui::SliderFloat("Camera rotation", &camera_rot, 0.0f, 360.0f);
    ImGui::SliderFloat("Camera height", &camera_height, -10.0f, 20.0f);
    //ImGui::SliderFloat("Simulator delta",(float*) &delta_time, 0.0001, 0.01);
    ImGui::End();

    sMat44 cube_models[15] = {}, sphere_models[15] = {};
    sVector4 cube_colors[15] = {}, sphere_colors[15] = {};
    int cube_size = 0, sphere_size = 0;

    //std::cout << transforms[0].scale.x << " " << transforms[0].scale.y  << " <=== " << std::endl;
    // Rendering ====
    // Render shapes
    int sphere_count = 0;
    int cube_count = 0;

    for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
      if (!phys_instance.enabled[i])
        continue;
      if (phys_instance.shape[i] == SPHERE_COLLIDER) {
        phys_instance.transforms[i].get_model(&sphere_models[sphere_count]);
        sphere_colors[sphere_count++] = {0.0f, 1.0f, 0.0f, 1.0f};
      } else if (phys_instance.shape[i] == CUBE_COLLIDER) {
        phys_instance.transforms[i].get_model(&cube_models[cube_count]);
        sphere_colors[cube_count++] = {0.0f, 0.0f, 1.0f, 1.0f};
      }
    }

    cube_renderer.render(cube_models, cube_colors, cube_count, proj_mat, true);
    sphere_renderer.render(sphere_models, sphere_colors, sphere_count, proj_mat, true);

    // Render contact points
    sVector4 col_color[15] = {};
    int col_points = 0;
    for(int i = 0; i < MAX_COLLISION_COUNT; i++) {
      if (!phys_instance.coll_manager.has_collided_on_frame[i])
        continue;

      for(int j = 0; j < phys_instance.coll_manager.manifold[i].contact_count; j++) {
        cube_models[col_points].set_identity();
        cube_models[col_points].set_position(phys_instance.coll_manager.manifold[i].contact_point[j]);
        cube_models[col_points].set_scale({0.03f, 0.03f, 0.03f});
        col_color[col_points++] = {1.0f, 0.0f, 0.0f, 1.00f};
      }
    }


    /*cube_models[col_points].set_identity();
    cube_models[col_points].set_position(phys_instance.transforms[dynamic_cube].position);
    cube_models[col_points].set_scale({0.03f, 0.03f, 0.03f});
    col_color[col_points++] = {1.0f, 0.0f, 0.0f, 1.00f};
    cube_models[col_points].set_identity();
    cube_models[col_points].set_position(phys_instance.transforms[dynamic_cube1].position);
    cube_models[col_points].set_scale({0.03f, 0.03f, 0.03f});
    col_color[col_points++] = {1.0f, 0.0f, 0.0f, 1.00f};*/


    glDisable(GL_DEPTH_TEST);
    sphere_renderer.render(cube_models, col_color, col_points, proj_mat, false);
    glEnable(GL_DEPTH_TEST);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  phys_instance.clean();
}

int main() {
	if (!glfwInit()) {
		return EXIT_FAILURE;
	}
	
	// GLFW config
	glfwSetErrorCallback(temp_error_callback);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);	
	
	GLFWwindow* window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, WIN_NAME, NULL, NULL);

	glfwSetKeyCallback(window, key_callback);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	if (!window) {
		std::cout << "Error, could not create window" << std::endl; 
	} else {
		if (!gl3wInit()) {
      //IMGUI_CHECKVERSION();
      ImGui::CreateContext();
      ImGuiIO &io = ImGui::GetIO();
      // Platform IMGUI
      ImGui_ImplGlfw_InitForOpenGL(window, true);
      ImGui_ImplOpenGL3_Init("#version 130");
      ImGui::StyleColorsDark();
      draw_loop(window);
      //test_loop(window);
		} else {
			std::cout << "Cannot init gl3w" << std::endl;
		}
		
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}
