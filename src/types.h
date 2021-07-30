#ifndef __TYPES_H_
#define __TYPES_H_

#include <stdlib.h>

#include "math.h"
#include "geometry.h"

struct sTransform {
  sMat44 rotation_mat;
  sVector3 position = sVector3{};
 
  sVector3 scale = sVector3{1.0f, 1.0f, 1.0f};

  void set_rotation(const sQuaternion4 quat) {
   convert_quaternion_to_matrix(&quat, &rotation_mat);
  }

  // Set the transform in the new_ref's space
  void change_basis(const sTransform &new_ref) {
    sMat44 tmp;
    
    // Invert the rottaion matrix via a transpose
    new_ref.rotation_mat.transpose_to(&tmp);
    tmp.multiply(&rotation_mat);
    memcpy(&rotation_mat, &tmp, sizeof(sMat44));

    // Invert the position
    tmp.set_identity();
    tmp.set_position(new_ref.position.invert());
    position = tmp.multiply(position);
  }

  inline sVector3 apply(const sVector3 vect) const {
    sMat44 mat;
    mat.set_position(position);
    return rotation_mat.multiply(mat.multiply(vect));
  }
};


struct sRawGeometry {
  sVector3  scale; 

  sVector3  *raw_points;
  sPlane    *planes;

  int *face_indexes;

  bool is_cube = false;

  unsigned int raw_point_size = 0;
  unsigned int planes_size = 0;
  unsigned int points_per_plane = 0;

  void get_face_vertices(const int       face_id, 
                               sVector3  *vertices) const {
    for(int i = 0; i < points_per_plane; i++) {
      //vertices[i] = raw_points[face_indexes[face_id][i]]; 
    }
  }

  void init_cuboid(const sVector3 &c_scale) {
    // Indexes of each face
    int box_LUT_vertices[6 * 4] = {
      4, 5, 7, 6,
      6, 7, 3, 2,
      0, 2, 6, 4,
      0, 1, 3, 2,
      0, 1, 5, 4,
      1, 3, 7, 5
    };

    scale = c_scale;
    face_indexes = (int*) malloc(sizeof(int) * 6 * 4);
    memcpy(face_indexes, box_LUT_vertices, sizeof(box_LUT_vertices));

    // Raw pointers
    raw_points = (sVector3*) malloc(sizeof(sVector3) * 8);
    raw_points[0] = sVector3{0.0f, 0.0f, 0.0f};
    raw_points[1] = sVector3{scale.x, 0.0f, 0.0f};
    raw_points[2] = sVector3{0.0f, scale.y, 0.0f};
    raw_points[3] = sVector3{scale.x, scale.y, 0.0f};
    raw_points[4] = sVector3{0.0f, 0.0f, scale.z};
    raw_points[5] = sVector3{scale.x, 0.0f, scale.z};
    raw_points[6] = sVector3{0.0f, scale.y, scale.z};
    raw_points[7] = sVector3{scale.x, scale.y, scale.z};

    // Generate planes
    planes = (sPlane*) malloc(sizeof(sPlane) * 6);
    for(int i = 0; i < 6; i++) {
      sVector3 center = {};

      for(int j = 0; j < 4; j++) {
        sVector3 tmp = raw_points[ box_LUT_vertices[(i * 4) + j] ];
        center.x += tmp.x;
        center.y += tmp.y;
        center.z += tmp.z;
      }

      center.x /= 4.0f;
      center.y /= 4.0f;
      center.z /= 4.0f;

      planes[i].origin_point = center;
    }
    // Plane orientations
    planes[0].normal = sVector3{0.0f, 1.0f, 0.0f};
    planes[1].normal = sVector3{0.0f, 0.0f, 1.0f};
    planes[2].normal = sVector3{1.0f, 0.0f, 0.0f};
    planes[3].normal = sVector3{0.0f, -1.0f, 0.0f};
    planes[4].normal = sVector3{0.0f, 0.0f, -1.0f};
    planes[5].normal = sVector3{-1.0f, 0.0f, 0.0f};

    raw_point_size = 8;
    planes_size = 6;
    points_per_plane = 4;
    is_cube = true; // This signalizes to only test 3 axis, instead of all the surface
                    // normals
  };

  void init_plane();

  void clean() {
    // TODO 
  };
};

#endif
