#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "Eigen/Core"


class Attitude
{
public:
    Attitude();
    bool set_C_from_obj_to_ref(float c11, float c12, float c13,
                               float c21, float c22, float c23,
                               float c31, float c32, float c33);
    bool set_C_from_ref_to_obj(float c11, float c12, float c13,
                               float c21, float c22, float c23,
                               float c31, float c32, float c33);
    bool set_psi_from_ref_to_obj(float psi_in);
    bool set_phi_from_ref_to_obj(float phi_in);
    bool set_theta_from_ref_to_obj(float theta_in);
    bool set_euler_from_ref_to_obj(float phi_in, float theta_in, float psi_in);

    float get_psi_from_ref_to_euler();
    float get_phi_from_ref_to_euler();
    float get_theta_from_ref_to_euler();

protected:
    Eigen::Matrix3f C_from_obj_to_ref;
    Eigen::Matrix3f C_from_ref_to_obj;
    float phi_from_ref_to_obj;
    float theta_from_ref_to_obj;
    float psi_from_ref_to_obj;

    bool update_euler_from_ref_to_obj();
    bool update_C();
};

#endif // ATTITUDE_H
