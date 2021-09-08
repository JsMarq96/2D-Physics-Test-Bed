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
                                 const sVector3  &axis) {
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

  return (shape1_len + shape2_len) > total_shapes_len;
}

inline sVector3 get_support_point_on_dir(const sVector3  &direction,
                                         const sVector3  *vertices,
                                         const int        vertices_count) {
  sVector3 support = {};
  float support_proj = -FLT_MAX;
  for(int i = 0; i < vertices_count; i++) {
    float proj = dot_prod(vertices[i], direction);

    if (proj > support_proj) {
      support = vertices[i];
      support_proj = proj;
    }
  }

  return support;
}

inline float get_max_distance_in_planes_axis(const sPlane    *obj1_planes,
                                             const int        obj1_planes_size,
                                             const sVector3  *obj2_vertices,
                                             const int        obj2_vertices_size,
                                                   int        *obj1_most_reparation_plane) {
  int most_separation_plane = -1;
  float most_separation = -FLT_MAX;
  for(int i = 0; i < obj1_planes_size; i++) {
    sVector3 obj2_support = get_support_point_on_dir(obj1_planes[i].normal.invert(), 
                                                     obj2_vertices,
                                                     obj2_vertices_size);

    float plane_support_distance = obj1_planes[i].distance(obj2_support);

    if (plane_support_distance > most_separation) {
      most_separation_plane = i;
      most_separation = plane_support_distance;
    }
  }

  *obj1_most_reparation_plane = most_separation_plane;
  return most_separation;
}

/* Note: SAT probably does not work since we change the aspect ratio,
 *the separating angles does not coincide, so we need to remove the scale out of the
 *transforms */

inline bool SAT_test(const sTransform   &obj1_transform,
                     const sTransform   &obj2_transform,
                           sCollisionManifold    *manifold) {
  // We evaluate the collisions in obj1's local space
  sTransform new_transf = obj1_transform.inverse().multiply(obj2_transform);

  //ImGui::Text("%f %f %f", new_transf.scale.x, new_transf.scale.y, new_transf.scale.z);

  sRawGeometry obj1 = {};
  obj1.init_cuboid(obj1_transform.scale);

  sRawGeometry obj2_in_obj1_space = {};
  obj2_in_obj1_space.init_cuboid(obj2_transform.scale);
  obj2_in_obj1_space.apply_transform(new_transf);

  //sVector3 p0 = obj1_transform.apply({0.f, 0.0f, 0.0f});
  //sVector3 p1 = obj1_transform.apply({1.f, 1.0f, 1.0f});
  sVector3 p0 = new_transf.apply({0.f, 0.0f, 0.0f});
  sVector3 p1 = new_transf.apply({1.f, 1.0f, 1.0f});


  //ImGui::Text("0 %f %f %f", p0.x, p0.y, p0.z);
  //ImGui::Text("1 %f %f %f", p1.x, p1.y, p1.z);

  // ========= SAT =================
  
  // First, we evaluate the planes and directions of obj1 vs the points of obj2
  int obj1_face_of_most_separation = -1;
  float obj1_nearest_distance = get_max_distance_in_planes_axis(obj1.planes, 
                                                                obj1.planes_size,
                                                                obj2_in_obj1_space.raw_points,
                                                                obj2_in_obj1_space.vertices_size,
                                                                &obj1_face_of_most_separation);

  // The nearest point is outside of the figure, so no collisions
  if (obj1_nearest_distance > 0.0f) {
    return false;
  }

  // Now, evaluate the points of obj1 vs the planes and direction of obj2
  int obj2_face_of_most_separation = -1;
  float obj2_nearest_distance = get_max_distance_in_planes_axis(obj2_in_obj1_space.planes, 
                                                                obj2_in_obj1_space.planes_size,
                                                                obj1.raw_points,
                                                                obj1.vertices_size,
                                                                &obj2_face_of_most_separation);
  
  if (obj2_nearest_distance > 0.0f) {
    return false;
  }
 

  // ======= Manifold generation =================

  // First we select the incident and the reference faces
  // the refrence face is the one whose normal has the axiss of
  // more penetrarion, and the incident is the most facing
  // face of the other body
  int incident_index, reference_index;
  const sRawGeometry *incident_obj, *reference_obj;

  if (obj1_nearest_distance + 1e-6f < obj2_nearest_distance) {
    reference_obj = &obj1;
    reference_index = obj1_face_of_most_separation;

    incident_obj = &obj2_in_obj1_space;
    //ImGui::Text("Incident 2");
  } else {
    reference_obj = &obj2_in_obj1_space;
    reference_index = obj2_face_of_most_separation;

    incident_obj = &obj1;
    //ImGui::Text("Incident 1");
  }

  // Calculate the incident face
  sVector3 reference_normal = reference_obj->planes[reference_index].normal;
  incident_index = -1;
  float incident_facing = FLT_MAX;
  for(int i = 0; i < incident_obj->planes_size; i++) {
    float facing = dot_prod(reference_normal, incident_obj->planes[i].normal);
    if (facing < incident_facing) {
      incident_index = i;
      incident_facing = facing;
    }
  }
 
  // Sutherland-Hgdman clipping ====
  /*
   * Use the Swapable stacks for iterating the vertices and one for
   * storing the modified ones, and then swp then back, for iterating the
   * previusly modified vertices, and storing on the (cleaned) old iterating buffer
   * */
  int incident_vertex_count = obj1.points_per_plane;

  sSwapableVector3Stacks swaps;

  swaps.init(incident_obj->points_per_plane * 3); 

  // Add to the stack the points 
  for(int i = 0; i < incident_obj->points_per_plane; i++) {
    sVector3 tmp = incident_obj->get_point_of_face(incident_index, i);
    //ImGui::Text(" col point %f %f %f / dist : %f", tmp.x, tmp.y, tmp.z, 0.0f);
    swaps.add_element_to_current_stack(incident_obj->get_point_of_face(incident_index, i));
  }


  for(int i = 0; i < reference_obj->planes_size; i++) { 
    sPlane *curr_plane = &reference_obj->planes[i];
   
    int element_count = swaps.get_current_stacks_size(); 
    for(int j = 0; j < element_count; j++) {
      sVector3 begin = swaps.get_element_from_current_stack(j);
      sVector3 end = swaps.get_element_from_current_stack((j+1) % element_count);

      float begin_dist = curr_plane->distance(begin);
      float end_dist = curr_plane->distance(end);

      if (begin_dist * end_dist > 1e-6f) {
        // If both have the same sign, and the begin distance is negative,
        // both are inside, if not, both are inside
        if (begin_dist < 1e-6f) {
          // Add the end point
          swaps.add_element_to_secundary_stack(end);
        }
      } else {
        // If the sing is negative, then the points are in different sides of
        // the plane
        
        // Add the intersection point
        swaps.add_element_to_secundary_stack(curr_plane->get_intersection_point(end, begin));
        if (begin_dist > 1e-6f) {
          // Add the end point
          swaps.add_element_to_secundary_stack(end);
        }
      }
    }

    swaps.clean_current_stack(); 
    swaps.swap(); 
  }
  //ImGui::Text("Colision points count:  %d", swaps.get_current_stacks_size());

  // Add the collision points to the manifold
  sPlane reference_plane = reference_obj->planes[reference_index];
  for(int j = 0; j < swaps.get_current_stacks_size(); j++) {
    sVector3 tmp = swaps.get_element_from_current_stack(j);

    ImGui::Text("%f %f %f", tmp.x, tmp.y, tmp.z);

    float distance = reference_plane.distance(tmp);
    tmp = obj1_transform.apply_without_scale(tmp);
    ImGui::Text("%f %f %f", tmp.x, tmp.y, tmp.z);

    manifold->add_collision_point(tmp, distance);
    ImGui::Separator();
  }

  ImGui::Text("ENDOL ====");
  manifold->collision_normal = reference_plane.normal;

  swaps.clean(); 
  obj2_in_obj1_space.clean();
  return true;
}

#endif
