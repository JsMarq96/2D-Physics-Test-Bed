#include "half_edge.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>


void sHalfEdgeMesh::load_OBJ_mesh(const char *mesh_dir) {
    FILE *mesh_file;
    int mesh_file_size;
    char *raw_mesh;

    mesh_file = fopen(mesh_dir, "r");

    assert(mesh_file != NULL && "Failed to open the mesh file");

    // First coutn the faces & vertices
    int read_chars;
    char *line_buffer = NULL;
    size_t len = 0;

    while((read_chars = getline(&line_buffer, &len, mesh_file)) != -1) {
        if (line_buffer[0] == 'v' && line_buffer[1] == ' ') {
            vertex_count++;
        } else if (line_buffer[0] == 'f') {
            face_count++;
        }
    }

    // 3 edges per face, and 3 half edges per face
    half_edge_count = face_count * 3;

    // 3 floats per edge
    indexing_count = vertex_count * 3;

    // Allocate memmory
    vertices_index = (uint16_t*) malloc(indexing_count * sizeof(uint16_t));
    vertices = (sRawVertex*) malloc(vertex_count * sizeof(sRawVertex));
    half_edges = (sHalfEdge*) malloc(half_edge_count * sizeof(sHalfEdge));
    face_normals = (sVector3*) malloc(face_count * sizeof(sVector3));

    sVector3 *temp_normals = (sVector3*) malloc(vertex_count * sizeof(sVector3));

    // TODO: maybe start from the top is not the best..?
    rewind(mesh_file);

    struct sUV_Wrapper {
        float u;
        float v;
    };

    int vertex_index = 0;
    int normal_count = 0;
    int half_edge_index = 0;
    int uv_count = 0;
    int faces_index = 0;
    sUV_Wrapper *tmp_uvs = NULL;

    union uKey {
        struct {
            uint16_t v1;
            uint16_t v2;
        };

        char str[32];
        uKey(uint16_t x, uint16_t y) {
            if (x > y) {
                v1 = x, v2 = y;
            } else {
                v1 = y, v2 = x;
            }
        }
    };


    sKVStorage half_edge_map;
    KVS_init(&half_edge_map);

    while((read_chars = getline(&line_buffer, &len, mesh_file)) != -1) {
        if (line_buffer[0] == 'v' && line_buffer[1] == ' ') {
            float x, y, z;
            sscanf(line_buffer,"v %f %f %f\n",&x, &y, &z);
            vertices[vertex_index].x = x;
            vertices[vertex_index].y = y;
            vertices[vertex_index].z = z;
            vertex_index++;
        } else if (line_buffer[0] == 'v' && line_buffer[1] == 't') {
            if (uv_count == 0) {
                tmp_uvs = (sUV_Wrapper*) malloc(sizeof(sUV_Wrapper) * (vertex_count));
            }
            float u,v;
            sscanf(line_buffer, "vt %f %f\n", &u, &v);

            // Store the UVs by tuples
            tmp_uvs[uv_count].u = u;
            tmp_uvs[uv_count].v =  1.f - v;
            uv_count++;
        } else if (line_buffer[0] == 'v' && line_buffer[1] == 'n') {
            sVector3 *curr_normal = &temp_normals[normal_count];
            sscanf(line_buffer, "vn %f %f %f\n", &curr_normal->x, &curr_normal->y, &curr_normal->z);

            normal_count++;
        } else if (line_buffer[0] == 'f') {
            int index1, index2, index3, normal1, normal2, normal3, uv1, uv2, uv3;
            sscanf(line_buffer,
                   "f %i/%i/%i %i/%i/%i %i/%i/%i\n",
                   &index1,
                   &uv1,
                   &normal1,
                   &index2,
                   &uv2,
                   &normal2,
                   &index3,
                   &uv3,
                   &normal3);

            index1 -= 1;
            index2 -= 1;
            index3 -= 1;
            uv1    -= 1;
            uv2    -= 1;
            uv3    -= 1;
            normal1 -=1;
            normal2 -=1;
            normal3 -=1;

            // Compute the face normal
            face_normals[face_count].x = (temp_normals[normal1].x + temp_normals[normal2].x + temp_normals[normal3].x) / 3.0f;
            face_normals[face_count].y = (temp_normals[normal1].y + temp_normals[normal2].y + temp_normals[normal3].y) / 3.0f;
            face_normals[face_count].z = (temp_normals[normal1].z + temp_normals[normal2].z + temp_normals[normal3].z) / 3.0f;

            // Fill the half-edges
            half_edges[half_edge_index].vertex_1 = index1;
            half_edges[half_edge_index].vertex_2 = index2;
            half_edges[half_edge_index].next = half_edge_index+1;
            half_edges[half_edge_index].face = face_count;

            // If there is an already stored vertex, create teh association
            uKey half_edge_key = uKey(index1, index2);
            int result = KVS_get_int(&half_edge_map, half_edge_key.str, 32);
            if (result == -1) {
                KVS_add(&half_edge_map, half_edge_key.str, 32, half_edge_index);
            } else {
                half_edges[half_edge_index].twin = result;
                half_edges[result].twin = half_edge_index;
            }

            half_edge_index++;

            half_edges[half_edge_index].vertex_1 = index2;
            half_edges[half_edge_index].vertex_2 = index3;
            half_edges[half_edge_index].next = half_edge_index+1;
            half_edges[half_edge_index].face = face_count;

            half_edge_key = uKey(index2, index3);
            result = KVS_get_int(&half_edge_map, half_edge_key.str, 32);
            if (result == -1) {
                KVS_add(&half_edge_map, half_edge_key.str, 32, half_edge_index);
            } else {
                half_edges[half_edge_index].twin = result;
                half_edges[result].twin = half_edge_index;
            }

            half_edge_index++;

            half_edges[half_edge_index].vertex_1 = index3;
            half_edges[half_edge_index].vertex_2 = index1;
            half_edges[half_edge_index].next = half_edge_index-2;
            half_edges[half_edge_index].face = face_count;

            half_edge_key = uKey(index3, index1);
            result = KVS_get_int(&half_edge_map, half_edge_key.str, 32);
            if (result == -1) {
                KVS_add(&half_edge_map, half_edge_key.str, 32, half_edge_index);
            } else {
                half_edges[half_edge_index].twin = result;
                half_edges[result].twin = half_edge_index;
            }

            half_edge_index++;

            // Add the UVs
            vertices[index1].u = tmp_uvs[uv1].u;
            vertices[index1].v = tmp_uvs[uv1].v;

            vertices[index2].u = tmp_uvs[uv2].u;
            vertices[index2].v = tmp_uvs[uv2].v;

            vertices[index3].u = tmp_uvs[uv3].u;
            vertices[index3].v = tmp_uvs[uv3].v;

            face_count++;
        }
    }

    free(tmp_uvs);

    fclose(mesh_file);
    // TODO: cleanup for the KVs
}
