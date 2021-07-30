#ifndef __COL_TESTING_H_
#define __COL_TESTING_H_

#include "math.h"
#include "geometry.h"
#include "types.h"
#include "collision_types.h"

#include "data_structs/swapable_stack.h"

#include "imgui/imgui.h"

inline bool test_overlap_on_axis(const sVector3  *shape1_vertices, 
                                 const int       shape1_size,
                                 const sVector3  *shape2_vertices,
                                 const int       shape2_size,
                                 const sVector3  &axis,
                                       float     *diff,
                                       bool      *is_obj1_first) {
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

  *is_obj1_first =  min_shape1 > min_shape2;

  return (shape1_len + shape2_len) > total_shapes_len;
}


inline bool SAT_test(const sRawGeometry &obj1, 
                     const sTransform   &obj1_transform,
                     const sRawGeometry &obj2,
                     const sTransform   &obj2_transform,
                           sCollisionManifold    *manifold) {
  // We evaluate the collisions in obj1's local space
  sTransform new_transf = obj2_transform;
  new_transf.change_basis(obj1_transform);

  sVector3 *obj1_vertices = obj1.raw_points;
  sVector3 *obj2_vertices = (sVector3*) malloc(obj2.raw_point_size * sizeof(sVector3));

  for(int i = 0; i < obj2.raw_point_size; i++) {
    obj2_vertices[i] = new_transf.apply(obj2.raw_points[i]);
  }

  float test_diff = 0.0f;
  bool is_obj1_first = false;
  
  int separating_axis1_index = -1;
  float axis1_diff = -FLT_MAX;
  int obj1_axis_count = (obj1.is_cube) ? 3 : obj1.planes_size;
  for(int i = 0; i < obj1_axis_count; i++) {
    if (!test_overlap_on_axis(obj1_vertices, obj1.raw_point_size, 
                              obj2_vertices, obj2.raw_point_size,
                              obj1.planes[i].normal,
                              &test_diff,
                              &is_obj1_first)) {
      free(obj2_vertices);
      return false;
    }

    if (test_diff > axis1_diff) {
      separating_axis1_index = i;
      axis1_diff = test_diff;

      if (is_obj1_first) {
        separating_axis1_index += 3;
      }
    }
  }

  int separating_axis2_index = -1;
  float axis2_diff = -FLT_MAX;
  int obj2_axis_count = (obj2.is_cube) ? 3 : obj2.planes_size;
  for(int i = 0; i < obj2_axis_count; i++) {
    if (!test_overlap_on_axis(obj1_vertices, obj1.raw_point_size, 
                              obj2_vertices, obj2.raw_point_size,
                              obj2.planes[i].normal,
                              &test_diff,
                              &is_obj1_first)) {
      free(obj2_vertices);
      return false;
    }

    if (test_diff > axis2_diff) {
      separating_axis2_index = i;
      axis2_diff = test_diff;

      if(!is_obj1_first) {
        separating_axis2_index += 3;
      }

    }
  }

  // Generate manifold
  // The reference face is aways on the object 1
  // Is this stable...?
  // TODO: rotate the planes

  sPlane reference_plane  = obj1.planes[separating_axis1_index];
  int incident_index = -1;
  float most_facing = FLT_MAX;
  for(int i = 0; i < obj2.planes_size; i++) {
    float dot = dot_prod(reference_plane.normal, obj2.planes[i].normal);

    if (most_facing > dot) {
      most_facing = dot;
      incident_index = i;
    }
  }
  sPlane incident_face = obj2.planes[incident_index];

  sVector3 *incident_vertices = (sVector3*) malloc(sizeof(sVector3) * obj2.points_per_plane);

  // Retrieve thje incident faces's points 
  for(int i = 0; i < obj2.points_per_plane; i++) {
    incident_vertices[i] = obj2_vertices[obj2.face_indexes[incident_index * obj2.points_per_plane + i]]; 
  }

  // Sutherland-Hgdman clipping ====
  /*
   * Use the Swapable stacks for iterating the vertices and one for
   * storing the modified ones, and then swp then back, for iterating the
   * previusly modified vertices, and storing on the (cleaned) old iterating buffer
   * */
  int incident_vertex_count = obj2.points_per_plane;

  sVector3 *vertices_to_read = (sVector3*) malloc(sizeof(sVector3) * 6);
  sVector3 *store_vertices = (sVector3*) malloc(sizeof(sVector3) * 6);

  memcpy(vertices_to_read, incident_vertices, sizeof(sVector3) * obj2.points_per_plane);

  int size_vertices_to_read = obj2.points_per_plane;
  int size_stored_vertices = 0;

  for(int i = 0; i < obj1.planes_size; i++) {

    for(int j = 0; j < size_vertices_to_read; j++) {
      sVector3 segment_begin = vertices_to_read[j];
      sVector3 segment_end = vertices_to_read[(j + 1) % size_vertices_to_read];

      bool is_outside;
      obj1.planes[i].clip_segment(&segment_begin, &segment_end, &is_outside);

      if (is_outside) {
        continue;
      } 
      store_vertices[size_stored_vertices++] = segment_end;
    }

    // Swap buffers
    {
      ImGui::Text("Stored %d", size_stored_vertices);
      memset(vertices_to_read, 0, sizeof(sVector3) * size_vertices_to_read);
      sVector3 *tmp = vertices_to_read;
      vertices_to_read = store_vertices;
      store_vertices = tmp;

      size_vertices_to_read = size_stored_vertices;
      size_stored_vertices = 0;
    } 
    ImGui::Separator();
    //manifold->add_collision_point(obj1_transform.apply(obj1.planes[i].origin_point), 0.0f); 
  } 

  for(int i = 0; i < size_vertices_to_read; i++) {
    manifold->add_collision_point(obj1_transform.apply(vertices_to_read[i]), 0.0f);
  } 
  
  //manifold->add_collision_point(obj1_transform.apply(obj1.planes[1].origin_point), -1.0f);
  //manifold->add_collision_point(obj1_transform.apply(obj1.planes[4].origin_point), -1.0f);

  ImGui::Text("Obj 1 axis: %d diff %f refernce index: %d", separating_axis1_index, axis1_diff, separating_axis1_index);
  ImGui::Text("Obj 2 axis: %d diff %f incident index %d", separating_axis2_index, axis2_diff, incident_index);
  ImGui::Text("Collision points:");
  for(int i = 0; i < manifold->contact_point_count; i++) {
    ImGui::Text("  %f %f %f", manifold->contact_points[i].x, manifold->contact_points[i].y, manifold->contact_points[i].z);
  }
  free(obj2_vertices);
  free(incident_vertices);
  free(store_vertices);
  free(vertices_to_read);
  return true;
}

#endif
