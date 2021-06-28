#ifndef _RENDER_CUBES_H_
#define _RENDER_CUBES_H_

#include <iostream>

#include "shader.h"

struct sCubeRenderer {
    unsigned int   VAO = -1;
    unsigned int   VBO = -1;
    unsigned int   EBO = -1;

    sShader shader;
};


inline void cube_renderer_init(sCubeRenderer *renderer) {
    const float clip_vertex[] = {
	   1.0f, 1.0f, 0.0f,
	   1.0f, -1.0f, 0.0f,
	   -1.0f, -1.0f, 0.0f,
	   -1.0f, 1.0f, 0.0f
	};
	unsigned int indices[] = { 0, 1, 3, 1, 2, 3 };

    const char* vertex_shader = " \
        #version 410 \n\
        layout(location = 0) in vec3 a_pos; \n\
        uniform mat4 u_model; \n\
        uniform mat4 u_proj; \n\
        void main() { \n\
            gl_Position = u_proj * u_model * vec4(a_pos, 1.0); \n\
        \n} \
    ";

    const char* fragment_shader = " \
        #version 410 \n\
        layout(location = 0) out vec4 FragColor; \n\
        uniform vec4 u_color; \n\
        void main() { \n\
            FragColor = u_color; \n\
        \n} \
    ";

    glGenVertexArrays(1, &renderer->VAO);
	glGenBuffers(1, &renderer->VBO);
	glGenBuffers(1, &renderer->EBO);

	glBindVertexArray(renderer->VAO);

	glBindBuffer(GL_ARRAY_BUFFER, renderer->VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(clip_vertex), clip_vertex, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderer->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*) 0);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

    renderer->shader = sShader(vertex_shader, fragment_shader);
}


inline void cube_renderer_render(sCubeRenderer *renderer, sMat44 *models, sVector4 *colors, int obj_count, sMat44 *proj_mat) {
    glBindVertexArray(renderer->VAO);

    renderer->shader.activate();
    for(int i = 0; i < obj_count; i++) {
        renderer->shader.set_uniform_matrix4("u_proj", proj_mat);
        renderer->shader.set_uniform_matrix4("u_model", &models[i]);
        renderer->shader.set_uniform_vector("u_color", colors[i]);

        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        std::cout << "Draw cube with color R:" << colors[i].x << " G:" << colors[i].y << " B:" << colors[i].z << std::endl;
    }
    renderer->shader.deactivate();

    glBindVertexArray(0);
}


#endif