#ifndef RAW_SHADERS_H_
#define RAW_SHADERS_H_

static const char borring_vertex_shader[] = "#version 410 \n"
                                          "layout(location = 0) in vec3 a_v_pos;\n"
                                          "layout(location = 2) in vec2 a_v_uv;\n"
                                          "layout(location = 3) in vec3 a_v_normal;\n"

                                          "uniform mat4 u_model_mat;\n"
                                          "uniform mat4 u_view_proj;\n"
                                          "uniform vec4 u_color; \n"
                                          "void main()\n"
                                          "{\n"
                                          "	gl_Position = u_view_proj * ( u_model_mat * vec4( "
                                          "a_v_pos, 1.0 ) );\n"
                                          "}\n";

static const char borring_v_vertex_shader[] = "#version 410 \n"
                                          "layout(location = 0) in vec3 a_v_pos;\n"

                                          "uniform mat4 u_model_mat;\n"
                                          "uniform mat4 u_view_proj;\n"
                                          "uniform vec4 u_color; \n"
                                          "void main()\n"
                                          "{\n"
                                          "	gl_Position = u_view_proj * ( u_model_mat * vec4( "
                                          "a_v_pos, 1.0 ) );\n"
                                          "}\n";


static const char borring_frag_shader[] = "#version 410\n"
                                        "\n"
                                        "layout(location = 0) out vec4 out_color;\n"
                                        "uniform vec4 u_color; \n"
                                        "void main()\n"
                                        "{\n"
                                        "	out_color = vec4(u_color.xyz, 1.0);\n"
                                        "}\n";

#endif // RAW_SHADERS_H_
