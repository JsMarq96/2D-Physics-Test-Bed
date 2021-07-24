#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include "math.h"

#include "imgui/imgui.h"

struct sLineSegment {
    sVector3  p1;
    sVector3  p2;
};

struct sPlane {
    sVector3  origin_point = sVector3{};
    sVector3  normal       = sVector3{};

    inline float distance(const sVector3 p) const {
        return dot_prod(normal, sVector3{  p.x - origin_point.x,  p.y - origin_point.y, p.z - origin_point.z });
    }

    inline void apply_transform(const sMat44 *transf) {
      /*sMat44 rot_mat;
      memcpy(&rot_mat, transf, sizeof(sMat44));
      rot_mat.set_position(sVector3{0.0f, 0.0f, 0.0f});
*/
      origin_point = transf->multiply(origin_point);
      // normal = rotate_vector3;
      //TODO: rotate normal
    }

    inline bool intersect(const sVector3  &p1, 
                          const sVector3  &p2,
                                bool      *is_inside,
                                sVector3  *clipped_p1,
                                sVector3  *clipped_p2) const {
      float dist_p1 = distance(p1);
      float dist_p2 = distance(p2); 
      std::cout << "BRUH" << std::endl;

      // Fast exit, there is no internsection
      if (dist_p1 > 0.0f && dist_p2 > 0.0f) {
        *is_inside = false;
        return false;
      } else if (dist_p1 < 0.0f && dist_p2 < 0.0f) {
        *is_inside = true;
        return false;
      }

      return true;
    }

  inline void clip_segment(sVector3 *p1, 
                           sVector3 *p2) const {
      float dist_p1 = distance(*p1);
      float dist_p2 = distance(*p2); 

      // Fast exit, there is no internsection
      if (dist_p1 > 0.0f && dist_p2 > 0.0f) {
        return;
      } else if (dist_p1 < 0.0f && dist_p2 < 0.0f) {
        return;
      }

      float p1_p2_dist = sVector3{p1->x - p2->x, p1->y - p2->y, p1->z - p2->z}.magnitude();

      if (dist_p1 > 0.0f) {
        // The first point is outside, an the other is outside,
        // so we clip it 
        *p1 = LERP_3D(*p1, *p2, 1.0f - (dist_p1 / p1_p2_dist));
        //ImGui::Text("P1 %f %f %f dist %f  %f", p1.x, p1.y, p1.z, dist_p1, dist_p1 / p1_p2_dist);
        //ImGui::Text("P2 %f %f %f dist %f", p2.x, p2.y, p2.z, dist_p2);
      } else if (dist_p2 > 0.0f) {
        // The second point is outside, an the other is outside,
        // so we clip it 
        *p2 = LERP_3D(*p2, *p1, 1.0f - (dist_p2 / p1_p2_dist));
      } 
      //ImGui::Text("P1 %f %f %f dist %f", clipped_p1->x, clipped_p1->y, clipped_p1->z, dist_p1);
      //ImGui::Text("P2 %f %f %f dist %f", clipped_p2->x, clipped_p2->y, clipped_p2->z, dist_p2);
      //ImGui::Separator();
  }

};


#endif // __GEOMETRY_H_
