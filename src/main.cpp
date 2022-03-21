#include <cstdint>
#include <iostream>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

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


void draw_loop(GLFWwindow *window) {
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
  cube.load_OBJ_mesh("resources/cube.obj");

  sMeshRenderer sphere_renderer, cube_renderer;
  sphere_renderer.create_from_mesh(&sphere);
  cube_renderer.create_from_mesh(&cube);

  sPhysWorld phys_instance;

  phys_instance.set_default_values();

  // Object 1: Static cube
  transforms[0].position = {0.0f, 0.0f, 0.0f};
  transforms[0].scale = {2.0f, 0.50f, 2.0f};
  transforms[0].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  phys_instance.mass[0] = 0.0f;
  phys_instance.restitution[0] = 0.25f;
  phys_instance.shape[0] = CUBE_COLLIDER;
  phys_instance.is_static[0] = true;
  phys_instance.enabled[0] = true;

  // Object 2: Dynamic sphere
  transforms[1].position = {0.07f, 7.0f, 0.0f};
  transforms[1].scale = {1.0f, 1.0f, 1.0f};
  transforms[1].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  phys_instance.restitution[1] = 0.6f;
  phys_instance.mass[1] = 15.0f;
  phys_instance.shape[1] = SPHERE_COLLIDER;
  phys_instance.enabled[1] = true;

  // Object 3: static plane
  transforms[3].position = {0.0f, -3.0f, 0.0f};
  transforms[3].scale = {1.0f, 1.0f, 1.0f};
  transforms[3].rotation = sQuaternion4{1.0f, 0.0f, 0.0f, 0.0f};
  phys_instance.restitution[3] = 0.15f;
  phys_instance.shape[3] = PLANE_COLLIDER;
  phys_instance.is_static[3] = true;
  phys_instance.enabled[3] = true;

  // Object 4: Static sphere 2
  transforms[2].position = {1.9f, 0.1f, 0.1f};
  transforms[2].scale = {2.0f, 2.0f, 2.0f};
  transforms[2].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  phys_instance.restitution[2] = 0.1f;
  phys_instance.mass[2] = 0.0f;
  phys_instance.shape[2] = SPHERE_COLLIDER;
  phys_instance.is_static[2] = true;
  phys_instance.enabled[2] = true;

  phys_instance.init(transforms);

  sVector4 colors[4] = {};
  colors[0] = {1.0f, 1.0f, 1.0f, 0.50f};
  colors[1] = {0.0f, 0.0f, 0.0f, 0.50f};
  colors[2] = {0.0f, 1.0f, 0.0f, 0.50f};
  colors[3] = {0.0f, 0.0f, 1.0f, 0.50f};

  float prev_frame_time = glfwGetTime();
  sCamera camera = {};
  float camera_rot = 0.0f;

  camera.position = {5.0f, 3.5f, 5.0f};

  // Frame counter
  int frames = 0;
  double start_time, fps;
  double delta_time = 0.01;
  double accumulator = 0.0;

  // Diagnostics
  float physics_ticks_per_frame[6] = {0.0f, 0.0f, 0.0f, 0.0f,};

  start_time = glfwGetTime();

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

    ImGui::Begin("Physics");
    ImGui::Text("FPS %f elapsed time %f", 1.0f / ( elapsed_time), elapsed_time);

    int num_of_physics_steps = 0;
    accumulator += elapsed_time;
    while(accumulator >= delta_time) {
      phys_instance.step(delta_time);
      accumulator -= delta_time;
      num_of_physics_steps++;
    }

    phys_instance.debug_speeds();
    ImGui::End();

    ImGui::Begin("Overall");
    ImGui::Text("Num of steps: %d", num_of_physics_steps);
    ImGui::End();

    sMat44 cube_models[15] = {}, sphere_models[15] = {};
    sVector4 cube_colors[15] = {}, sphere_colors[15] = {};
    int cube_size = 0, sphere_size = 0;

    // Rendering ====
    for(int i = 0; i < 3; i++) {
      if (phys_instance.shape[i] == SPHERE_COLLIDER) {
        sphere_colors[sphere_size] = colors[i];
        transforms[i].get_model(&sphere_models[sphere_size++]);
      } else if (phys_instance.shape[i] == CUBE_COLLIDER) {
        cube_colors[cube_size] = colors[i];
        transforms[i].get_model(&cube_models[cube_size++]);
      }
    }

    cube_renderer.render(cube_models, cube_colors, cube_size, proj_mat, true);
    sphere_renderer.render(sphere_models, sphere_colors, sphere_size, proj_mat, true);

    sVector4 col_color[15] = {};
    for(int i = 0; i < phys_instance.curr_frame_col_count; i++) {
      cube_models[i].set_position(phys_instance._manifolds[i].contact_points[0]);
      cube_models[i].set_scale({0.05f, 0.05f, 0.05f});
      col_color[i] = {1.0f, 0.0f, 0.0f, 0.90f};
    }

    glDisable(GL_DEPTH_TEST);
    sphere_renderer.render(cube_models, col_color, phys_instance.curr_frame_col_count, proj_mat, false);
    glEnable(GL_DEPTH_TEST);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }
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
      //test_draw_loop(window);
		} else {
			std::cout << "Cannot init gl3w" << std::endl;
		}
		
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}
