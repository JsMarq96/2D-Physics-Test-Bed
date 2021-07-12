//
// Created by jsmar on 16/06/2021.
//

#ifndef QUEST_DEMO_SAT_COLLISION_TESTING_H
#define QUEST_DEMO_SAT_COLLISION_TESTING_H

#include <float.h>

#include "imgui/imgui.h"

#include "math.h"
#include "geometry.h"

#include "collision_types.h"

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

  return (shape1_len + shape2_len) >= total_shapes_len;
}

inline bool SAT_test_OBB_v_OBB(const sBoxCollider &collider1,
                               const sBoxCollider &collider2) {

  ImGui::Text("Obj 1 %f %f %f", collider1.vertices[0].x, collider1.vertices[0].y, collider1.vertices[0].z);
  ImGui::Text("Obj 2 %f %f %f", collider2.vertices[0].x, collider2.vertices[0].y, collider2.vertices[0].z);

  // TODO: only test on 3 axis...? Check box2d or qu3b
  float collider1_max_dif = -FLT_MAX;
  for(int i = 0; i < 3; i++) {
    float diff;
    if (!test_overlap_on_axis(collider1.vertices, 8, 
                              collider2.vertices, 8, 
                              collider1.axis[i],
                              &diff)) {
      return false;
    }
    
    ImGui::Text("Col 1 axis: %f %f %f Diff: %f", collider1.axis[i].x, collider1.axis[i].y, collider1.axis[i].z, diff);
    collider1_max_dif = MAX(collider1_max_dif, diff);
  }

  float collider2_max_diff = -FLT_MAX;
  for(int i = 0; i < 3; i++) {
    float diff;
    if (!test_overlap_on_axis(collider1.vertices, 8, 
                              collider2.vertices, 8, 
                              collider2.axis[i],
                              &diff)) {
      return false;
    }
    
    ImGui::Text("Col 2 axis: %f %f %f Diff: %f", collider2.axis[i].x, collider2.axis[i].y, collider2.axis[i].z, diff);
    collider2_max_diff = MAX(collider2_max_diff, diff);
  }
 
  ImGui::Text("Colider 1 diff %f", collider1_max_dif);
  ImGui::Text("Collider 2 diff %f", collider2_max_diff);
  ImGui::Separator();

  return true;
}

#endif //QUEST_DEMO_SAT_COLLISION_TESTING_H
