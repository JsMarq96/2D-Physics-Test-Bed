#ifndef HALF_EDGE_H_
#define HALF_EDGE_H_


#include <cassert>
#include <cstdint>
#include <stdlib.h>
#include <string.h>


#include "../math.h"
#include "../kv_storage.h"

struct sHalfEdge {
    // Edge data
    uint16_t  vertex_1 = 0;
    uint16_t  vertex_2 = 0;

    // Half-edge data
    uint16_t  next = 0;
    uint16_t  twin = 0;
    uint16_t  face = 0;
};

inline void get_key_of_vertex(const uint16_t v1,
                              const uint16_t v2,
                              char result[5]) {
    // Using an union to map the vertex ids to a string
    union{ struct{ uint16_t x; uint16_t y;};
           char tmp[5];
    } key;

    // The highst is is always the x value
    if (v1 > v2) {
        key.x = v1;
        key.y = v2;
    } else {
        key.x = v2;
        key.y = v1;
    }

    // Return the result
    memcpy(result, key.tmp, sizeof(char[5]));
}

struct sRawVertex {
    union {
        sVector3 vertex = {0.0f, 0.0f, 0.0f};
        struct {
            float x;
            float y;
            float z;
        };
    };
    float u = 0.0f;
    float v = 0.0f;

    // TODO: per vertex normal..?
};

struct sHalfEdgeMesh {
    uint16_t    vertex_count    = 0;
    uint16_t    half_edge_count = 0;
    uint16_t    face_count      = 0;
    uint16_t    indexing_count  = 0;

    uint16_t    *vertices_index = NULL;

    union {
        sRawVertex  *vertices     = NULL;
        float       *raw_vertices;
    };

    sHalfEdge   *half_edges     = NULL;

    sVector3    *face_normals   = NULL;


    void load_OBJ_mesh(const char* mesh_dir);

    void clean();
};

#endif // HALF_EDGE_H_
