//
// Created by jsmar on 22/06/2021.
//

#ifndef QUEST_DEMO_COLLISION_TYPES_H
#define QUEST_DEMO_COLLISION_TYPES_H

#include "math.h"
#include "geometry.h"
#include "imgui/imgui.h"

#include <iostream>

struct sCollisionManifold {
    int obj1_index            = -1;
    int obj2_index            = -1;

    int face_obj1 = -1;
    int face_obj2 = -1;

    sVector3  collision_normal     = {};
    sVector3  contact_points    [6] = {};
    float     points_depth      [6] = {};
    int       contact_point_count   = 0;

    inline void add_collision_point(const sVector3& point, const float depth) {
      if (contact_point_count == 5) {
        return;
      }

      contact_points[contact_point_count] = point;
      points_depth[contact_point_count++] = depth;
    }
};

// Look up Table for the indices of each vertex on each face
// The vertices are ordered via neighboors, so the 
// lines that compose the face 0 are:
// 0-1, 1-3, 3-2 and 2-0
unsigned int BOX_3D_LUT_FACE_VERTICES[6][4] = {
  {0, 1, 3, 2},
  {6, 7, 3, 2},
  {0, 2, 6, 4},
  {4, 5, 7, 6},
  {0, 1, 5, 4},
  {1, 3, 7, 5}
};

struct sBoxCollider {
  sVector3   vertices[8] = { sVector3{0.0f, 0.0f, 0.0f},
                             sVector3{1.0f, 0.0f, 0.0f},
                             sVector3{0.0f, 1.0f, 0.0f},
                             sVector3{1.0f, 1.0f, 0.0f},
                             // NOTE: evaluate until here for 2D coldet
                             sVector3{0.0f, 0.0f, 1.0f},
                             sVector3{1.0f, 0.0f, 1.0f},
                             sVector3{0.0f, 1.0f, 1.0f},
                             sVector3{1.0f, 1.0f, 1.0f} };

  sVector3   axis[6]    = {  sVector3{0.0f, -1.0f, 0.0f}, // y pos
                             sVector3{0.0f, 0.0f, 1.0f}, // z pos
                             sVector3{1.0f, 0.0f, 0.0f}, // x pos
                             sVector3{0.0f, 1.0f, 0.0f},
                             sVector3{0.0f, 0.0f, -1.0f},
                             sVector3{-1.0f, 0.0f, 0.0f} };

  sPlane    planes[6];

  sMat44     transform;
  sQuaternion4 rotation;

  inline void modify(const sVector3      &position, 
                     const sVector3      &scale, 
                     const sQuaternion4  &rot) {
    for(int i = 0; i < 8; i++) {
      vertices[i].multiply(scale);
    }

    transform.rotate(&rot);
    transform.set_position(position);

    rotation = rot;
  }

  inline void modify(const sMat44 &model, 
                     const sQuaternion4 &rot) {
    transform = model;
    rotation = rot;  
  
    for(int i = 0; i < 6; i++) {
      axis[i] = rotate_vector3(axis[i], rot);
      planes[i] = get_plane_of_face(i, false);
    }

    for(int i = 0; i < 8; i++) {
      vertices[i] = model.multiply(vertices[i]); 
    } 
  }

  inline sVector3 get_point_of_face(const int point_id, 
                                    const int face_id) const {
    return vertices[BOX_3D_LUT_FACE_VERTICES[face_id][point_id]];
  }

  inline void get_planes_of_axis(const int axis,
                                 sPlane *plane1, 
                                 sPlane *plane2) const {
    switch(axis) {
      case 0: // Z axis
        *plane1 = get_plane_of_face(0, false);
        *plane2 = get_plane_of_face(3, false);
        break;
      case 1: // Y axis
        *plane1 = get_plane_of_face(1, false);
        *plane2 = get_plane_of_face(4, false);
        break;
      case 3: // X axis
        *plane1 = get_plane_of_face(2, false);
        *plane2 = get_plane_of_face(5, false);
        break;
    };
  }

  // TODO: a bit of cleanup would be nice
  inline sPlane get_plane_of_face(const int face_index, const bool local_coords) const {
    float avg_x = 0.0f, avg_y = 0.0f, avg_z = 0.0f;
    for(int i = 0; i < 4; i++) {
      sVector3 tmp = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][i]];

      tmp = transform.multiply(tmp);  

      avg_x += tmp.x;
      avg_y += tmp.y;
      avg_z += tmp.z;
    }

    return sPlane{sVector3{avg_x / 4.0f, avg_y / 4.0f, avg_z / 4.0f}, 
                  rotate_vector3( axis[face_index], rotation )};
  };

  inline void get_lines_of_face(const int face_index, 
                                sVector3  lines[4][2]) const {
    // Vert 1
    lines[0][0] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][0]];
    lines[0][1] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][1]];
    // Vert 2
    lines[1][0] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][1]];
    lines[1][1] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][2]];
    // Vert 3
    lines[2][0] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][2]];
    lines[2][1] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][3]];
    // Vert 4
    lines[3][0] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][3]];
    lines[3][1] = vertices[BOX_3D_LUT_FACE_VERTICES[face_index][0]];
  }

};

#endif //QUEST_DEMO_COLLISION_TYPES_H
