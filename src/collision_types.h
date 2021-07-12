//
// Created by jsmar on 22/06/2021.
//

#ifndef QUEST_DEMO_COLLISION_TYPES_H
#define QUEST_DEMO_COLLISION_TYPES_H

#include "math.h"
#include "geometry.h"

#include <iostream>

struct sCollisionManifold {
    int obj1_index            = -1;
    int obj2_index            = -1;

    sVector3  collision_normal     = {};
    sVector3  contact_points    [4] = {};
    float     points_depth      [4] = {};
    int       contact_point_count   = 0;
};

unsigned int BOX_3D_LUT_FACE_VERTICES[6][4] = {
  {0, 1, 2, 3},
  {6, 7, 2, 3},
  {6, 4, 0, 2},
  {6, 7, 4, 5},
  {4, 5, 0, 1},
  {7, 5, 3, 1}
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

  sVector3   axis[6]    = {  sVector3{0.0f, 0.0f, 1.0f},
                             sVector3{0.0f, 1.0f, 0.0f},
                             sVector3{1.0f, 0.0f, 0.0f},
                             sVector3{0.0f, 0.0f, 1.0f},
                             sVector3{0.0f, -1.0f, 0.0f},
                             sVector3{-1.0f, 0.0f, 0.0f} };
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
    for(int i = 0; i < 8; i++) {
      vertices[i] = model.multiply(vertices[i]);
    }

    transform = model;

    rotation = rot;
  }

  // TODO: a bit of cleanup would be nice
  inline sPlane get_plane_of_face(const int face_index, const bool local_coords) const {
    float avg_x = 0.0f, avg_y = 0.0f, avg_z = 0.0f;
    for(int i = 0; i < 4; i++) {
      sVector3 tmp = vertices[BOX_3D_LUT_FACE_VERTICES[i][face_index]];

      if (!local_coords) {
        tmp = transform.multiply(tmp); 
      } 

      avg_x += tmp.x;
      avg_y += tmp.y;
      avg_z += tmp.z;
    }

    if (!local_coords) {
      return sPlane{sVector3{avg_x / 4.0f, avg_y / 4.0f, avg_z / 4.0f}, 
                    rotate_vector3( axis[face_index], rotation )};
    } else {
      return sPlane{sVector3{avg_x / 4.0f, avg_y / 4.0f, avg_z / 4.0f}, 
                    axis[face_index]};

    }
  };
};

#endif //QUEST_DEMO_COLLISION_TYPES_H
