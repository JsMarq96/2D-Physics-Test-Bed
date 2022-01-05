#ifndef MESH_RENDERER_H_
#define MESH_RENDERER_H_

#include "gl3w.h"
#include "glcorearb.h"
#include "shader.h"
#include "raw_shaders.h"
#include "mesh.h"

#include <iostream>

struct sMeshRenderer {
    unsigned int  VAO = 0;
    unsigned int  VBO = 0;
    unsigned int  EBO = 0;

    uint16_t indices_count = 0;

    sShader  shader;

    void create_from_mesh(const sMesh *mesh) {
        indices_count = mesh->indexing_count;

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        // Load vertices
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, mesh->vertex_count * sizeof(sRawVertex), mesh->vertices, GL_STATIC_DRAW);

        // Vertex position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*) 0);

        // UV coords
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*) (sizeof(float) * 2));

        // Vertex normal
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*) (sizeof(float) * 3));

        // Load vertex indexing
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh->indexing_count * sizeof(uint16_t), mesh->vertices_index, GL_STATIC_DRAW);


        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindVertexArray(0);

        shader = sShader(borring_vertex_shader, borring_frag_shader);
    }

    void render(const sMat44 *models,
                const sVector4 *colors,
                const int count,
                const sMat44 &view_proj,
                const bool show_wireframe) const {
        std::cout << count << std::endl;

        glBindVertexArray(VAO);

        if (show_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        shader.activate();

        for(int i = 0; i < count; i++) {
            shader.set_uniform_matrix4("u_model_mat", &models[i]);
            shader.set_uniform_matrix4("u_view_proj", &view_proj);
            shader.set_uniform_vector("u_color", colors[i]);

            glDrawElements(GL_TRIANGLES, indices_count, GL_UNSIGNED_SHORT, 0);
        }

        shader.deactivate();
        glBindVertexArray(0);
    }

    void clean() {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }
};


#endif // MESH_RENDERER_H_
