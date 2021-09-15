#include <iostream>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include "shader.h"
#include "input_layer.h"
#include "render_cubes.h"
#include "camera.h"
#include "collision_testing.h"
#include "types.h"
#include "physics.h"
#include "collision_resolution.h"

// Dear IMGUI
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"

#define WIN_WIDTH	740
#define WIN_HEIGHT	680
#define WIN_NAME	"Test"


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

void test_draw_loop(GLFWwindow *window) {
  glfwMakeContextCurrent(window);

  // Generate geometry data

  sCubeRenderer renderer = {};

  cube_renderer_init(&renderer);

  sCamera camera = {};
  camera.position = {2.0f, 0.2f, 2.0f};

  double prev_frame_time = glfwGetTime();

  sTransform transf[2] = {{}};

  transf[0].position = {0.3f, 0.0f, -0.3f};
  transf[0].scale = {1.0f, 0.5f, 2.0f};
  transf[0].rotation = sQuaternion4{1.0f, 0.f, 0.0f, 0.0f};

  transf[1].position = {0.0f, 0.0f, 0.0f};
  transf[1].scale = {10.0f, 0.5f, 1.0f};
  transf[1].rotation = sQuaternion4{1.0f, 2.0f,0.0f, 0.0f}.normalize();

  transf[0] = transf[1].multiply( transf[1].inverse().multiply(transf[0]));

  sVector4 colors[3] = {{1.0f, 0.0f, 0.0f, 1.0f},
                        {0.0f, 1.0f, 0.0f, 1.0f},
                        {0.0f, 0.0f, 1.0f, 1.0f}};

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


    float camera_rot = 1.10f;
    camera.position = rotate_arround(camera.position,
                                     sVector3{0.0f, 0.0f, 0.0f},
                                     to_radians(camera_rot));

    camera.look_at(sVector3{0.0f, 0.0f, 0.0f});
    camera.get_perspective_viewprojection_matrix(90.0f,
                                                100.0f,
                                                0.1f,
                                                (float)width / (float)heigth,
                                                &proj_mat);

    double curr_frame_time = glfwGetTime();
    double elapsed_time = curr_frame_time - prev_frame_time;
    prev_frame_time = curr_frame_time;

    sMat44 model[3] = {};

    transf[0].get_model(&model[0]);
    transf[1].get_model(&model[1]);

    ImGui::Begin("Rot");
    for(int i = 0; i < 2; i++) {
      sQuaternion4 q = transf[i].rotation;
    }
    sVector3 t = transf[0].apply({1.0f, 0.0f, 1.0f});

    sCollisionManifold manifold = {};

    if (SAT_test(transf[0], transf[1], &manifold)) {
      ImGui::Text("Collision");
      sMat44 col_points[6] = {};
      sVector4 col_color[6] = {};

      for(int i = 0; i < manifold.contact_point_count; i++) {
        col_points[i].set_identity();
        col_points[i].set_scale({0.25, 1.05, 0.25});
        col_points[i].set_position(manifold.contact_points[i]);
        col_color[i] = {0.0f, 0.0f, 1.0f, 1.0f};
      }

      cube_renderer_render(&renderer, col_points, col_color, manifold.contact_point_count, &proj_mat);
    }

    model[2].set_identity();
    model[2].set_position(t);
    model[2].set_scale({0.05f, 0.05f, 0.05f});

    ImGui::End();

    cube_renderer_render(&renderer,
                         model,
                         colors,
                         2,
                         &proj_mat);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }
}

void draw_loop(GLFWwindow *window) {
	glfwMakeContextCurrent(window);
  char names[4][5] = {
    "WHIT",
    "BLAK",
    "GREN",
    "BLU"
  };
  sRawGeometry cubes[4] = {};
  sTransform   transforms[4] = {};

  transforms[0].position = {-5.0f, 0.0f, -5.0f};
  transforms[0].scale = {10.0f, 1.0f, 10.0f};
  transforms[0].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});

  transforms[1].position = {0.0f, 4.0f, 0.0f};
  transforms[1].scale = {1.0f, 1.0f, 1.0f};
  transforms[1].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  //transforms[1].rotation.set_identity();

  transforms[2].position = {0.7f, 5.5f, 0.0f};
  transforms[2].scale = {1.0f, 1.0f, 1.0f};
  transforms[2].rotation = sQuaternion4{1.0f, 0.0f, 0.0f, 0.0f};
  //transforms[2].rotation.set_identity();

  transforms[3].position = {1.5f, 4.0f, 3.5f};
  transforms[3].set_rotation({1.0f, 0.0f, 0.0f, 0.0f});
  //transforms[3].rotation.set_identity();

  sVector4 colors[4] = {};
  colors[0] = {1.0f, 1.0f, 1.0f, 1.0f};
  colors[1] = {0.0f, 0.0f, 0.0f, 1.0f};
  colors[2] = {0.0f, 1.0f, 0.0f, 1.0f};
  colors[3] = {0.0f, 0.0f, 1.0f, 1.0f};

  sCubeRenderer renderer;
  sPhysicsWorld phys_instance;
  phys_instance.transforms = &transforms[0];

  phys_instance.mass[0] = 0.0f;
  phys_instance.mass[1] = 29.0f;
  phys_instance.mass[2] = 11.0f;
  phys_instance.mass[3] = 9.0f;

  phys_instance.restitution[0] = 0.5f;
  phys_instance.restitution[1] = 0.3f;
  phys_instance.restitution[2] = 0.5f;
  phys_instance.restitution[3] = 0.2f;

  phys_instance.friction[0] = 0.8f;
  phys_instance.friction[1] = 0.5f;
  phys_instance.friction[2] = 0.5f;
  phys_instance.friction[3] = 0.7f;

  phys_instance.is_static[0] = true;

  for(int i = 0; i < 4; i++) {
    sVector3 curr_scale = transforms[i].scale;

    phys_instance.mass_center[i] = {curr_scale.x / 2.0f, curr_scale.y / 2.0f, curr_scale.z / 2.0f};
  }

  phys_instance.config_simulation();

  cube_renderer_init(&renderer);
  float prev_frame_time = glfwGetTime();
  sCamera camera = {};
  float camera_rot = 0.0f;

  camera.position = {5.0f, 3.5f, 5.0f};

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

    ImGui::Begin("Physics");
    phys_instance.step(elapsed_time);
    ImGui::End();

    sMat44 models[4] = {};

    for(int i = 0; i < 4; i++) {
      transforms[i].get_model(&models[i]);
      //ImGui::Text("Obj %d  %f %f %f", i,  transforms[i].position.x, transforms[i].position.y, transforms[i].position.z);
    }

    cube_renderer_render(&renderer, models, colors, 4, &proj_mat);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    //std::cout << "==== End Frame ====" << std::endl;

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
