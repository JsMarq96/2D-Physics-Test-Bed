#ifndef __COL_TESTING_H_
#define __COL_TESTING_H_

#include "math.h"
#include "geometry.h"
#include "types.h"

#include "imgui/imgui.h"

inline bool test_overlap_on_axis(const sVector3  *shape1_vertices, 
                                 const int       shape1_size,
                                 const sVector3  *shape2_vertices,
                                 const int       shape2_size,
                                 const sVector3  &axis,
                                       float     *diff) {
  float min_shape1 = FLT_MAX;
  float max_shape1 = -FLT_MAX;
  float min_shape2 = FLT_MAX;
  float max_shape2 = -FLT_MAX;

  // Get the maximun and minimun on the projections
  for(int i = 0; i < shape1_size; i++) {
    float proj = dot_prod(shape1_vertices[i], axis);

    min_shape1 = MIN(min_shape1, proj);
    max_shape1 = MAX(max_shape1, proj);
  }
  for(int i = 0; i < shape2_size; i++) {
    float proj = dot_prod(shape2_vertices[i], axis);

    min_shape2 = MIN(min_shape2, proj);
    max_shape2 = MAX(max_shape2, proj);
  }

  float shape1_len = max_shape1 - min_shape1;
  float shape2_len = max_shape2 - min_shape2;
  float total_shapes_len = MAX(max_shape1, max_shape2) - MIN(min_shape1, min_shape2);

  *diff = (shape1_len + shape2_len) - total_shapes_len;

  return (shape1_len + shape2_len) > total_shapes_len;
}


inline bool SAT_test(const sRawGeometry &obj1, 
                     const sTransform   &obj1_transform,
                     const sRawGeometry &obj2,
                     const sTransform   &obj2_transform) {
  // We evaluate the collisions in obj1's local space
  sTransform new_transf = obj2_transform;
  new_transf.change_basis(obj1_transform);

  sVector3 *obj1_vertices = obj1.raw_points;
  sVector3 *obj2_vertices = (sVector3*) malloc(obj2.raw_point_size * sizeof(sVector3));

  for(int i = 0; i < obj2.raw_point_size; i++) {
    obj2_vertices[i] = new_transf.apply(obj2.raw_points[i]);
    //obj2_vertices[i] = obj2.raw_points[i];
    //ImGui::Text("V %f %f %f", obj2_vertices[i].x, obj2_vertices[i].y, obj2_vertices[i].z);
  }
  //ImGui::Separator();

  int separating_axis1_index = -1;
  float axis1_diff = -FLT_MAX;
  for(int i = 0; i < obj1.planes_size; i++) {
    if (!test_overlap_on_axis(obj1_vertices, obj1.raw_point_size, 
                              obj2_vertices, obj2.raw_point_size,
                              obj1.planes[i].normal,
                              &axis1_diff)) {
      free(obj2_vertices);
      return false;
    }
  }

  int separating_axis2_index = -1;
  float axis2_diff = -FLT_MAX;
  for(int i = 0; i < obj2.planes_size; i++) {
    if (!test_overlap_on_axis(obj1_vertices, obj1.raw_point_size, 
                              obj2_vertices, obj2.raw_point_size,
                              obj2.planes[i].normal,
                              &axis2_diff)) {
      free(obj2_vertices);
      return false;
    }
  }

  // Generate manifold

  free(obj2_vertices);

  return true;
}

#endif
