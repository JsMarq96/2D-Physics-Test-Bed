#include <iostream>
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include "shader.h"
#include "input_layer.h"
#include "render_cubes.h"
#include "camera.h"
#include "SAT_collision_testing.h"

// Dear IMGUI
#include "imgui/imgui.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_impl_glfw.h"

#define WIN_WIDTH	640
#define WIN_HEIGHT	480
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

void get_projection_matrix(sMat44 *result, float vp_width, float vp_height) {
    result->set_identity();

    float left = (-vp_width) * 0.06f / 2.0f;
    float right = (vp_width) * 0.06f / 2.0f;
    float bottom = (-vp_height) * 0.06f / 2.0f;
    float top = (vp_height) * 0.06f / 2.0f;

    result->mat_values[0][0] = 2.0f / (right - left);
    result->mat_values[1][1] = 2.0f / (top - bottom);
    result->mat_values[2][2] = -1.0f;
    result->mat_values[3][0] = -(right + left) / (right - left);
    result->mat_values[3][1] = -(top + bottom) / (top - bottom);
}

void draw_loop(GLFWwindow *window) {
	glfwMakeContextCurrent(window);

  sVector3 positions[6] = {};
  sVector3 scales[6] = {};

  scales[0] = {1.0f, 1.0f, 1.0f};
  scales[1] = {1.0f, 1.0f, 1.0f};

  positions[0] = {0.5, .5, 0.0f};
  positions[1] = {-1.5, 1.5, 0.0f};

	sMat44 models[6];
	sVector4 colors[6] = {{}};
	sQuaternion4 rotations[6] = {{}};

	rotations[0] = {1.0f, 0.0f, 0.0f, 0.0f};
	rotations[1] = {1.0f, 0.0f, 0.0f, 0.0f};

	models[0].set_scale({1.0f, 1.0f, 1.0f});
	models[0].rotate(&rotations[0]);
	models[0].set_position({0.5, .5, 0.0f});
	colors[0] = {1.0f, 1.0f, 1.0f, 1.0f};

	//models[1].set_scale({4.15, 1.05, 1.0f});
  models[1].set_scale({1.0f, 1.0f, 1.0f});
	models[1].rotate(&rotations[1]);
	models[1].set_position({-1.5, 1.5, 0.0f});
	colors[1] = {0.0f, 1.0f, .0f, 1.0f};

	sCubeRenderer renderer;
	cube_renderer_init(&renderer);

	double prev_frame_time = glfwGetTime();
	sInputLayer *input_state = get_game_input_instance();

  sCamera camera;

  //camera.right = sVector3{0.0f, 0.0f, 3.0f};
  //camera.up = sVector3{0.0f, 0.f, 0.f};
  //camera.forward = sVector3{0.0f, 1.0f, 0.f};
  camera.position = sVector3{5.0f, 5.0f, 5.0f}; 

	while(!glfwWindowShouldClose(window)) {
		// Draw loop
		int width, heigth;
		double temp_mouse_x, temp_mouse_y;

    glfwPollEvents();
		glfwGetFramebufferSize(window, &width, &heigth);
		// Set to OpenGL viewport size anc coordinates
		glViewport(0,0, width, heigth);

		sMat44 proj_mat;	

		// OpenGL stuff
    glEnable(GL_DEPTH_TEST);  
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

		double curr_frame_time = glfwGetTime();
		double elapsed_time = curr_frame_time - prev_frame_time;

    if (models[1].px > 3.0f) {
      models[1].px = -3.0f;
      positions[1].x = -3.0f;
    } else {
      models[1].px += 0.005f;
      positions[1].x += 0.005f;
    }
  
    camera.look_at(sVector3{0.0f, 0.0f, 0.0f});
    camera.get_perspective_viewprojection_matrix(90.0f, 
                                                1000.0f,
                                                0.001f,
                                                (float)width / (float)heigth,
                                                &proj_mat);
    //camera.look_at(positions[1]);
		// Mouse position control
		glfwGetCursorPos(window, &temp_mouse_x, &temp_mouse_y);

    ImGui::Begin("Position data");
    for (int i = 0; i < 2; i++) {
      ImGui::Text("Obj: %i", i);
      ImGui::Text("Pos: x: %f y: %f z: %f", positions[i].x, positions[i].y,positions[i].z);
      ImGui::Text("Rot: %f %f %f %f", rotations[i].x, rotations[i].y, rotations[i].z, rotations[i].w);
      ImGui::Separator();
    }
    ImGui::End();
		// Test collisions
    ImGui::Begin("Collision data");
		for(int i = 0; i < 2; i++) {
			for(int j = i; j < 2; j++) {
				if (i == j) {
					continue;
				}
        sBoxCollider col1, col2;
        col1.modify(models[i], 
                    rotations[i]);

        col2.modify(models[j], 
                    rotations[j]);

        if (SAT_test_OBB_v_OBB(col1, col2)) {
          std::cout << "Collision detected" << std::endl;
          //ImGui::Text("Collision between %i and %i", i, j);
        } else {
          std::cout << "No collision detected" << std::endl;
        }
			}
		}
    ImGui::End();

    ImGui::Begin("Proj mat");
    for (int i = 0; i < 4; i++) {
      ImGui::Text("%f %f %f %f", 
          proj_mat.mat_values[i][0], 
          proj_mat.mat_values[i][1], 
          proj_mat.mat_values[i][2], 
          proj_mat.mat_values[i][3]);
    }
    ImGui::End();

		cube_renderer_render(&renderer, models, colors, 2, &proj_mat);

		std::cout << " ======== Frame end ======= " << std::endl; 

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
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorEnterCallback(window, cursor_enter_callback);

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	std::cout << "test" << std::endl;
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
		} else {
			std::cout << "Cannot init gl3w" << std::endl;
		}
		
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}
