#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include "math.h"

#include "imgui/imgui.h"

struct sEdge {
    sVector3  p1;
    sVector3  p2;
};

struct sPlane {
    sVector3  origin_point = sVector3{};
    sVector3  normal       = sVector3{};

    inline float distance(const sVector3 p) const {
        return dot_prod(normal, sVector3{  p.x - origin_point.x,  p.y - origin_point.y, p.z - origin_point.z });
    }

    inline bool intersect(const sVector3  &p1, 
                          const sVector3  &p2,
                                bool      *is_inside,
                                sVector3  *clipped_p1,
                                sVector3  *clipped_p2) const {
      float dist_p1 = distance(p1);
      float dist_p2 = distance(p2); 

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
                           sVector3 *p2,
                           bool     *is_outside) const {
      float dist_p1 = distance(*p1);
      float dist_p2 = distance(*p2); 
      //ImGui::Text("%f %f", dist_p1, dist_p2); 
      if ((dist_p1 * dist_p2) > 1e-6f) {
        *is_outside = (dist_p1) > 1e-6f;
        //ImGui::Text("END");
        return;
      }
     
      sVector3 p1_p2 = sVector3{p1->x - p2->x, p1->y - p2->y, p1->z - p2->z};
      float p1_p2_dist = p1_p2.magnitude();

      p1_p2.x /= p1_p2_dist;
      p1_p2.y /= p1_p2_dist;
      p1_p2.z /= p1_p2_dist;

      float cos_theta = dot_prod(normal, p1_p2);
      
      sVector3 intersect = {p2->x - p1_p2.x * (dist_p2 / cos_theta), 
                        p2->y - p1_p2.y * (dist_p2 / cos_theta),
                        p2->z - p1_p2.z * (dist_p2 / cos_theta)
      };

      if (dist_p1 > 1e-6f) {
        *p1 = intersect;
      } else {
        *p2 = intersect;
      }

  }

  inline sVector3 get_intersection_point(const sVector3 &p1, 
                                         const sVector3 &p2) const {
      float dist_p1 = distance(p1);
      float dist_p2 = distance(p2); 
     
      sVector3 p1_p2 = sVector3{p1.x - p2.x, p1.y - p2.y, p1.z - p2.z};
      float p1_p2_dist = p1_p2.magnitude(); 

      p1_p2.x /= p1_p2_dist;
      p1_p2.y /= p1_p2_dist;
      p1_p2.z /= p1_p2_dist;

      // Avoid that pesky division by 0
      float cos_theta = dot_prod(normal, p1_p2) + 1e-6f;
      
      sVector3 intersect = {p1.x - p1_p2.x * (dist_p1 / cos_theta), 
                        p1.y - p1_p2.y * (dist_p1 / cos_theta),
                        p1.z - p1_p2.z * (dist_p1 / cos_theta)
      };

      return intersect;
  }

  inline sVector3 project_point(const sVector3 &point) const {
    float point_distance = distance(point);

    return sVector3 {point.x - point_distance * normal.x, 
                     point.y - point_distance * normal.y,
                     point.z - point_distance * normal.z};
  } 

};


#endif // __GEOMETRY_H_
