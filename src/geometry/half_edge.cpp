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

    int uv_total_count = 0;
    int normal_total_count = 0;

    while((read_chars = getline(&line_buffer, &len, mesh_file)) != -1) {
        if (line_buffer[0] == 'v' && line_buffer[1] == ' ') {
            vertex_count++;
        } else if (line_buffer[0] == 'f') {
            face_count++;
        } else if (line_buffer[0] == 'v' && line_buffer[1] == 't') {
            uv_total_count++;
        }else if (line_buffer[0] == 'v' && line_buffer[1] == 'n') {
            normal_total_count++;
        }
    }
    //free(line_buffer);

    // 3 edges per face, and 3 half edges per face
    half_edge_count = face_count * 3;

    // 3 floats per edge
    //indexing_count = face_count * 3;

    // Allocate memmory
    vertices_index = (uint16_t*) malloc(face_count * 3 * sizeof(uint16_t));
    vertices = (sRawVertex*) malloc(face_count * 3 * sizeof(sRawVertex));
    half_edges = (sHalfEdge*) malloc(half_edge_count * sizeof(sHalfEdge));
    face_normals = (sVector3*) malloc(face_count * sizeof(sVector3));

    struct sUV_Wrapper {
        float u;
        float v;
    };


    sVector3 *temp_normals = (sVector3*) malloc(normal_total_count * sizeof(sVector3));
    sUV_Wrapper *tmp_uvs = (sUV_Wrapper*) malloc(sizeof(sUV_Wrapper) * (uv_total_count));

    // TODO: maybe start from the top is not the best..?
    rewind(mesh_file);

    int vertex_index = 0;
    int normal_count = 0;
    int half_edge_index = 0;
    int uv_count = 0;
    int face_index = 0;


    sKVStorage half_edge_map;
    half_edge_map.init();

    sKVStorage vertex_map;
    vertex_map.init();

    char tmp[5];
    char tmp_edge_id[12];

    while((read_chars = getline(&line_buffer, &len, mesh_file)) != -1) {
        if (line_buffer[0] == 'v' && line_buffer[1] == ' ') {
            float x, y, z;
            sscanf(line_buffer,"v %f %f %f\n",&x, &y, &z);
            vertices[vertex_index].x = x;
            vertices[vertex_index].y = y;
            vertices[vertex_index].z = z;
            vertex_index++;
        } else if (line_buffer[0] == 'v' && line_buffer[1] == 't') {
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

            // Vertex 1
            get_key_of_vertex(index1, normal1, uv1, tmp_edge_id);
            int vertex_id = vertex_map.get_int(tmp_edge_id, 12);
            if (vertex_id == -1) {
                vertices[index1].normal = temp_normals[normal1];
                vertices[index1].u = tmp_uvs[uv1].u;
                vertices[index1].v = tmp_uvs[uv1].v;
                vertex_id = index1;
                vertex_map.add(tmp_edge_id, 12, index1);
            }
            vertices_index[indexing_count++] = vertex_id;

            // Vertex 2
            get_key_of_vertex(index2, normal2, uv2, tmp_edge_id);
            vertex_id = vertex_map.get_int(tmp_edge_id, 12);
            if (vertex_id == -1) {
                vertices[index2].normal = temp_normals[normal2];
                vertices[index2].u = tmp_uvs[uv2].u;
                vertices[index2].v = tmp_uvs[uv2].v;
                vertex_id = index2;
                vertex_map.add(tmp_edge_id, 12, index2);
            }
            vertices_index[indexing_count++] = vertex_id;

            // Vertex 3
            get_key_of_vertex(index3, normal3, uv3, tmp_edge_id);
            vertex_id = vertex_map.get_int(tmp_edge_id, 12);
            if (vertex_id == -1) {
                vertices[index3].normal = temp_normals[normal3];
                vertices[index3].u = tmp_uvs[uv3].u;
                vertices[index3].v = tmp_uvs[uv3].v;
                vertex_id = index3;
                vertex_map.add(tmp_edge_id, 12, index3);
            }
            vertices_index[indexing_count++] = vertex_id;

            // Compute the face normal
            face_normals[face_index].x = (temp_normals[normal1].x + temp_normals[normal2].x + temp_normals[normal3].x) / 3.0f;
            face_normals[face_index].y = (temp_normals[normal1].y + temp_normals[normal2].y + temp_normals[normal3].y) / 3.0f;
            face_normals[face_index].z = (temp_normals[normal1].z + temp_normals[normal2].z + temp_normals[normal3].z) / 3.0f;

            // Fill the half-edges
            half_edges[half_edge_index].vertex_1 = index1;
            half_edges[half_edge_index].vertex_2 = index2;
            half_edges[half_edge_index].next = half_edge_index+1;
            half_edges[half_edge_index].face = face_count;

            get_key_of_edge(index1, index2, tmp);
            int edge_id = half_edge_map.get_int(tmp, 5);
            // If tehre is no record, create one
            // if there is, the link both edges
            if (edge_id == -1) {
                half_edge_map.add(tmp, 5, half_edge_index);
            } else {
                half_edges[half_edge_index].twin = edge_id;
                half_edges[edge_id].twin = half_edge_index;
            }

            half_edge_index++;

            half_edges[half_edge_index].vertex_1 = index2;
            half_edges[half_edge_index].vertex_2 = index3;
            half_edges[half_edge_index].next = half_edge_index+1;
            half_edges[half_edge_index].face = face_count;

            get_key_of_edge(index2, index3, tmp);
            edge_id = half_edge_map.get_int(tmp, 5);
            // If tehre is no record, create one
            // if there is, the link both edges
            if (edge_id == -1) {
                half_edge_map.add(tmp, 5, half_edge_index);
            } else {
                half_edges[half_edge_index].twin = edge_id;
                half_edges[edge_id].twin = half_edge_index;
            }

            half_edge_index++;

            half_edges[half_edge_index].vertex_1 = index3;
            half_edges[half_edge_index].vertex_2 = index1;
            half_edges[half_edge_index].next = half_edge_index-2;
            half_edges[half_edge_index].face = face_count;

            get_key_of_edge(index3, index1, tmp);
            edge_id = half_edge_map.get_int(tmp, 5);
            // If tehre is no record, create one
            // if there is, the link both edges
            if (edge_id == -1) {
                half_edge_map.add(tmp, 5, half_edge_index);
            } else {
                half_edges[half_edge_index].twin = edge_id;
                half_edges[edge_id].twin = half_edge_index;
            }

            half_edge_index++;
            face_count++;
        }
    }

    free(line_buffer);
    free(tmp_uvs);
    free(temp_normals);

    fclose(mesh_file);
    half_edge_map.clean();
    vertex_map.clean();
    // TODO: cleanup for the KVs
}


void sHalfEdgeMesh::clean() {
    free(vertices_index);
    free(vertices);
    free(half_edges);
    free(face_normals);

    vertex_count = 0;
    half_edge_count = 0;
    face_count = 0;
    indexing_count = 0;
}
