#include <cstdint>
#include <iostream>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include "geometry/half_edge.h"
#include "glcorearb.h"
#include "shader.h"
#include "input_layer.h"
#include "render_cubes.h"
#include "camera.h"
#include "types.h"

#include "physics.h"

// Dear IMGUI
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"

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

	eKeyMaps pressed_key;
	switch(key) {
		case GLFW_KEY_W:
			pressed_key = W_KEY;
			break;
		case GLFW_KEY_A:
			pressed_key = A_KEY;
			break;
		case GLFW_KEY_S:
			pressed_key = S_KEY;
			break;
		case GLFW_KEY_D:
			pressed_key = D_KEY;
			break;
		case GLFW_KEY_UP:
			pressed_key = UP_KEY;
			break;
		case GLFW_KEY_DOWN:
			pressed_key = DOWN_KEY;
			break;
		case GLFW_KEY_RIGHT:
			pressed_key = RIGHT_KEY;
			break;
		case GLFW_KEY_LEFT:
			pressed_key = LEFT_KEY;
			break;

		sInputLayer *input = get_game_input_instance();
		input->keyboard[pressed_key] = (action == GLFW_PRESS) ? KEY_PRESSED : KEY_RELEASED;
	};


}

void mouse_button_callback(GLFWwindow *wind, int button, int action, int mods) {
	char index;

	switch (button) {
	case GLFW_MOUSE_BUTTON_LEFT:
		index = LEFT_CLICK;
		break;

	case GLFW_MOUSE_BUTTON_RIGHT:
		index = RIGHT_CLICK;
		break;

	case GLFW_MOUSE_BUTTON_MIDDLE:
		index = MIDDLE_CLICK;
		break;
	}

	sInputLayer *input = get_game_input_instance();
	input->mouse_clicks[index] = (action == GLFW_PRESS) ? KEY_PRESSED : KEY_RELEASED;
}

void cursor_enter_callback(GLFWwindow *window, int entered) {
	sInputLayer *input = get_game_input_instance();
	input->is_mouse_on_screen = entered;
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

#include "kv_storage.h"
void test_draw_loop(GLFWwindow *window) {
  //glfwMakeContextCurrent(window);

   sHalfEdgeMesh cube;

  cube.load_OBJ_mesh("resources/cube.obj");
  cube.clean();
  return;
}

void draw_loop(GLFWwindow *window) {
	glfwMakeContextCurrent(window);
  char names[4][5] = {
    "WHIT",
    "BLAK",
    "GREN",
    "BLU"
  };
  sRawGeometry cubes[6] = {};
  sTransform   transforms[6] = {};


  sHalfEdgeMesh cube;

  cube.load_OBJ_mesh("resources/cube.obj");

  sPhysWorld phys_instance;

  phys_instance.set_default_values();

  // Object 1: Static sphere
  transforms[0].position = {0.0f, 1.0f, 0.0f};
  transforms[0].scale = {2.0f, 2.0f, 2.0f};
  transforms[0].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  phys_instance.mass[0] = 0.0f;
  phys_instance.restitution[0] = 0.25f;
  phys_instance.shape[0] = SPHERE_COLLIDER;
  phys_instance.is_static[0] = true;
  phys_instance.enabled[0] = true;

  // Object 2: Dynamic sphere
  transforms[1].position = {0.07f, 4.0f, 0.0f};
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

  sCubeRenderer renderer;

  cube_renderer_init(&renderer);
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

    sMat44 models[4] = {};

    // Rendering ====
    for(int i = 0; i < 3; i++) {
      transforms[i].get_model(&models[i]);
      //ImGui::Text("Obj %d  %f %f %f", i,  transforms[i].position.x, transforms[i].position.y, transforms[i].position.z);
    }

    cube_renderer_render(&renderer, models, colors, 3, &proj_mat);

    sVector4 col_color[4] = {};
    for(int i = 0; i < phys_instance.curr_frame_col_count; i++) {
      models[i].set_position(phys_instance._manifolds[i].contact_points[0]);
      models[i].set_scale({0.05f, 0.05f, 0.05f});
      col_color[i] = {1.0f, 0.0f, 0.0f, 0.90f};
    }

    glDisable(GL_DEPTH_TEST);
    cube_renderer_render(&renderer, models, col_color, phys_instance.curr_frame_col_count, &proj_mat);
    glEnable(GL_DEPTH_TEST);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }
}

int main() {
  test_draw_loop(NULL);
  return 0;
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
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorEnterCallback(window, cursor_enter_callback);

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
