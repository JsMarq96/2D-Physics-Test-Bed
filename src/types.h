#ifndef __TYPES_H_
#define __TYPES_H_

#include <stdlib.h>

#include "math.h"
#include "geometry.h"

struct sTransform {
  sQuaternion4 rotation_quat = {1.0f, 0.0f, 0.0f, 0.0f};
  sMat44 rotation_mat;
  sVector3 position = sVector3{};
 
  sVector3 scale = sVector3{1.0f, 1.0f, 1.0f};


  //https://math.stackexchange.com/questions/1693067/differences-between-quaternion-integration-methods 
  void integrate_angular_speed(const sVector3 angular_speed, 
                               const float    delta_time) {
    sQuaternion4 qr = {0.0f, angular_speed.x, angular_speed.y, angular_speed.z};

    qr = qr.multiply(delta_time);

    rotation_quat = rotation_quat.sum(rotation_quat.multiply(qr).multiply(0.5f));
    rotation_quat.normalize();
    convert_quaternion_to_matrix(&rotation_quat, &rotation_mat);
    //std::cout << rotation_quat.w << " " << rotation_quat.x << " " << rotation_quat.y << " " << rotation_quat.z << std::endl;;
    //rotation_mat.print();
  }

  void set_rotation(const sQuaternion4 quat) {
    rotation_quat = quat;
    convert_quaternion_to_matrix(&quat, &rotation_mat);
  }

  // Set the transform in the new_ref's space
  void change_basis(const sTransform &new_ref) {
    sMat44 tmp = {};
    
    // Invert the rottaion matrix via a transpose
    new_ref.rotation_mat.transpose_to(&tmp);
    tmp.multiply(&rotation_mat);
    memcpy(&rotation_mat, &tmp, sizeof(sMat44));

    // Invert the position
    tmp.set_identity();
    tmp.set_position(new_ref.position.invert());
    position = tmp.multiply(position);
  }

  inline void apply(sVector3 *vect) const {
    sMat44 mat;
    mat.set_position(position);
    mat.set_scale(scale);
    *vect = rotation_mat.multiply(mat.multiply(*vect));
  }

  inline sVector3 apply(const sVector3 &vect) const {
    sMat44 mat;
    mat.set_position(position);
    mat.set_scale(scale);
    return rotation_mat.multiply(mat.multiply(vect));
  }

  inline void apply(sPlane *plane) const {
    apply(&(plane->origin_point));
    plane->normal = rotation_mat.multiply(plane->normal);
  }
};


struct sRawGeometry {
  sVector3  scale; 

  sVector3  *raw_points;
  sPlane    *planes;

  int *face_indexes;
  // TODO: generalize the face indexes and add w,h if needed

  bool is_cube = false;

  unsigned int vertices_size = 0;
  unsigned int planes_size = 0;
  unsigned int points_per_plane = 0;

  void init_cuboid(const sVector3 &c_scale) {
    // Indexes of each face
    int box_LUT_vertices[6 * 4] = {
      4, 5, 7, 6, // 0
      6, 7, 3, 2, // 1
      1, 3, 7, 5, // 5
      0, 1, 3, 2, // 3
      0, 1, 5, 4, // 4 
      0, 2, 6, 4 // 2
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
    planes[0].normal = sVector3{0.0f, 0.0f, 1.0f};
    planes[1].normal = sVector3{0.0f, 1.0f, 0.0f};
    planes[2].normal = sVector3{1.0f, 0.0f, 0.0f};
    planes[3].normal = sVector3{0.0f, 0.0f, -1.0f};
    planes[4].normal = sVector3{0.0f, -1.0f, 0.0f};
    planes[5].normal = sVector3{-1.0f, 0.0f, 0.0f};

    vertices_size = 8;
    planes_size = 6;
    points_per_plane = 4;
    is_cube = true; // This signalizes to only test 3 axis, instead of all the surface
                    // normals
  };

  void duplicate(sRawGeometry *copy_to) const {
    copy_to->scale = scale;
    copy_to->is_cube = is_cube;
    copy_to->vertices_size = vertices_size;
    copy_to->planes_size = planes_size;
    copy_to->points_per_plane = points_per_plane; 

    copy_to->face_indexes = (int*) malloc(sizeof(int) * planes_size * points_per_plane);
    copy_to->raw_points = (sVector3*) malloc(sizeof(sVector3) * vertices_size);
    copy_to->planes = (sPlane*) malloc(sizeof(sPlane) * planes_size);

    memcpy(copy_to->face_indexes, face_indexes, sizeof(int) * planes_size * points_per_plane);
    memcpy(copy_to->raw_points, raw_points, sizeof(sVector3) * vertices_size);
    memcpy(copy_to->planes, planes, sizeof(sPlane) * planes_size);
  }

  /*
   * Calculates a support point in a given direction
   * A support point is the furcest point in that direction
   * */
  inline sVector3 get_support_point(const sVector3 &direction) const {
    sVector3 support;
    float support_proj = -FLT_MAX;

    for(int i = 0; i < vertices_size; i++) {
      float proj = dot_prod(raw_points[i], direction);

      if (proj > support_proj) { 
        support = raw_points[i];
        support_proj = proj;
      }
    }

    return support;
  }

  inline sVector3 get_point_of_face(const int face,
                                    const int point) const {
    int index = face_indexes[face * points_per_plane + point];
    return raw_points[index];
  }

  inline void apply_transform(const sTransform &transf) {
    for(int i = 0; i < vertices_size; i++) {
      transf.apply(&raw_points[i]);
    }

    for(int i = 0; i< planes_size; i++) {
      transf.apply(&planes[i]);
    }
  }

  void clean() {
    free(face_indexes);
    free(raw_points);
    free(planes);

    // TODO: finish, but laterrr
  };
};

#endif
