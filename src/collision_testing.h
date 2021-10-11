#ifndef __COL_TESTING_H_
#define __COL_TESTING_H_

#include "math.h"
#include "geometry.h"
#include "types.h"
#include "collision_types.h"
#include "clipping.h"

#include "imgui/imgui.h"

#include <cstdint>

enum eColType : uint8_t {
  FACE_FACE = 0,
  FACE_VERTEX,
  VERTEX_VERTEX
};

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

  sRawGeometry obj1 = {};
  obj1.init_cuboid(obj1_transform);
  //obj1.init_cuboid(sVector3{1.0f, 1.0f, 1.0f});

  sRawGeometry obj2_in_obj1_space = {};
  //obj2_in_obj1_space.init_cuboid(new_transf);
  obj2_in_obj1_space.init_cuboid(obj2_transform);
  //obj2_in_obj1_space.init_cuboid(obj2_transform);
  //obj2_in_obj1_space.apply_transform(new_transf);

  // ========= SAT =================

  eColType collision_type = FACE_FACE;
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

  // TODO: VERTEX-VERTEX TEST
 

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

    manifold->reference_index = manifold->obj1_index;
    manifold->incident_index = manifold->obj2_index;
    //ImGui::Text("Incident 2");
  } else {
    reference_obj = &obj2_in_obj1_space;
    reference_index = obj2_face_of_most_separation;

    incident_obj = &obj1;

    manifold->reference_index = manifold->obj2_index;
    manifold->incident_index = manifold->obj1_index;
    //ImGui::Text("Incident 1");
  }

  // Calculate the incident face
  // TODO: Fill reference normal
  sVector3 reference_normal = {};// = reference_obj->planes[reference_index].normal;
  incident_index = -1;
  float incident_facing = -FLT_MAX;
  for(int i = 0; i < incident_obj->planes_size; i++) {
    float facing = dot_prod(reference_normal, incident_obj->planes[i].normal);
    if (facing > incident_facing) {
      incident_index = i;
      incident_facing = facing;
    }
  }

  //manifold->add_collision_point(incident_obj->planes[incident_index].origin_point, 0.0f, {0,0});

  switch(collision_type) {
    case FACE_FACE:
      plain_plain_clipping(reference_index,
                           incident_index,
                           reference_obj,
                           incident_obj,
                           obj1_transform,
                           manifold);
      break;
    case FACE_VERTEX: break; // TODO
    case VERTEX_VERTEX: break; // TODO
  }



  // Add the faces to the manifold
  manifold->incident_face = incident_index;
  manifold->reference_face = reference_index;
 
  // Sutherland-Hgdman clipping ====
    int incident_vertex_count = obj1.points_per_plane;


  //ImGui::Text("ENDOL ====");
  manifold->collision_normal = reference_normal;

  obj1.clean();
  obj2_in_obj1_space.clean();
  return true;
}

#endif
