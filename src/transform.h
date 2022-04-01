#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include "math.h"
#include "geometry.h"

struct sTransform {
  sQuaternion4 rotation = {1.0f, 0.0f, 0.0f, 0.0f};
  sVector3 position = sVector3{};
  sVector3 scale = sVector3{1.0f, 1.0f, 1.0f};


  //https://math.stackexchange.com/questions/1693067/differences-between-quaternion-integration-methods
  void set_rotation(const sQuaternion4 quat) {
    rotation = quat;
  }

  void rotate(const sQuaternion4 quat) {
    rotation = quat.multiply(rotation);
    rotation = rotation.normalize();
  }

  sTransform inverse() const {
    return sTransform{ rotation.inverse(), position.mult(-1.0f), sVector3{1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z}};
  }

  sTransform multiply(const sTransform &b) const {
    sTransform res = {};

    res.rotation = rotation.multiply(b.rotation);
    res.position = position.sum(b.position);
    res.scale = scale.mult(b.scale);

    return res;
  }

  inline sVector3  apply_without_scale(const sVector3 &vect) const {
    return rotation.inverse().multiply(vect.get_pure_quaternion()).multiply(rotation).get_vector().sum(position);
  }

  inline sVector3 apply_rotation(const sVector3 &vect) const {
    return rotation.inverse().multiply(vect.get_pure_quaternion()).multiply(rotation).get_vector();
  }

  inline sVector3 apply(const sVector3 &vect) const {
    sVector3 scalled = vect.mult(scale);//
    sQuaternion4 q_vect = scalled.get_pure_quaternion();

    // Using a' = Q * a * Q^-1
    return rotation.inverse().multiply(q_vect).multiply(rotation).get_vector().sum(position).subs(scale.mult(0.50f));
  }

  // Generate model matrix
  inline void get_model(sMat44 *mat) const {
    sMat44 scale_mat = {}, rot_mat = {}, cent_mat = {};

    // Order of transforms: Position <- Rotation <- Scale
    mat->set_identity();
    mat->set_position(position.subs(scale));

    scale_mat.set_identity();
    scale_mat.set_scale(scale);

    rot_mat.convert_quaternion_to_matrix(rotation);

    rot_mat.multiply(&scale_mat);

    mat->multiply(&rot_mat);
    mat->add_position(scale);
  }
};


#endif // TRANSFORM_H_
