#ifndef CLIPPING_H_
#define CLIPPING_H_

#include "math.h"
#include "types.h"
#include "collision_types.h"
#include "data_structs/swapable_stack.h"

/**
 * Functions to perform clipping on the collision detecion
 * */


// Using sutherland hogman clipping between the two faces
inline void plain_plain_clipping(const int reference_face,
                                 const int incident_face,
                                 const sRawGeometry *reference_obj,
                                 const sRawGeometry *incident_obj,
                                 const sTransform   &result_transform,
                                 sCollisionManifold *manifold) {
    /*
   * Use the Swapable stacks for iterating the vertices and one for
   * storing the modified ones, and then swp then back, for iterating the
   * previusly modified vertices, and storing on the (cleaned) old iterating buffer
   * */
    sSwapableVector3Stacks swaps;
  // In order to store the IDS of the points
  sSwapableVector3Stacks mirror_ids_swaps;

  swaps.init(incident_obj->points_per_plane * 3);
  mirror_ids_swaps.init(incident_obj->points_per_plane * 3);

  // Add to the stack the points
  for(int i = 0; i < incident_obj->points_per_plane; i++) {
    sVector3 tmp = incident_obj->get_point_of_face(incident_face, i);
    swaps.add_element_to_current_stack(incident_obj->get_point_of_face(incident_face, i));
    mirror_ids_swaps.add_element_to_current_stack(sVector3{(float)i, (float)i, 0.0});
  }

  for(int i = 0; i < reference_obj->neighbor_faces_per_face; i++) {
        sPlane *curr_plane = reference_obj->get_neighboring_plane(reference_face, i);

    int element_count = swaps.get_current_stacks_size();
    for(int j = 0; j < element_count; j++) {
      int end_index = (j+1) % element_count;
      sVector3 begin = swaps.get_element_from_current_stack(j);
      sVector3 end = swaps.get_element_from_current_stack(end_index);

      float begin_dist = curr_plane->distance(begin);
      float end_dist = curr_plane->distance(end);

      if (begin_dist * end_dist > 0.0f) {
        // If both have the same sign, and the begin distance is negative,
        // both are inside, if not, both are outside
        if (begin_dist < 1e-6f) {
          // Add the end point
          swaps.add_element_to_secundary_stack(end);
          mirror_ids_swaps.add_element_to_secundary_stack({(float)end_index, (float)end_index});
        } else {
            // both outside
          }
      } else {
        // If the sing is negative, then the points are in different sides of
        // the plane

        // Add the intersection point
        swaps.add_element_to_secundary_stack(curr_plane->get_intersection_point(end, begin));
        mirror_ids_swaps.add_element_to_secundary_stack({(float)end_index, (float)i});

        if (begin_dist > 1e-6f) {
          // Add the end point
          swaps.add_element_to_secundary_stack(end);
          mirror_ids_swaps.add_element_to_secundary_stack({(float)end_index, (float)end_index});
        }
      }
    }
    swaps.clean_current_stack();
    swaps.swap();
    mirror_ids_swaps.clean_current_stack();
    mirror_ids_swaps.swap();
  }

  // Add the collision points to the manifold
  sPlane reference_plane = reference_obj->planes[reference_face];
  for(int j = 0; j < swaps.get_current_stacks_size(); j++) {
    sVector3 tmp = swaps.get_element_from_current_stack(j);
    sVector3 ids = mirror_ids_swaps.get_element_from_current_stack(j);

    float distance = reference_plane.distance(tmp);
    tmp = result_transform.apply_without_scale(tmp);

    manifold->add_collision_point(tmp, distance, {(uint16_t) ids.x, (uint16_t) ids.y});
  }

  swaps.clean();

}
#endif // CLIPPING_H_
