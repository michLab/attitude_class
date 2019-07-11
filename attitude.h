#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "Eigen/Core"

#define ATTITUDE_VERSION    "Attitude v.0.1.0.0"


class Attitude
{
public:
    Attitude();
    bool set_C_a_b(float c11, float c12, float c13,
                               float c21, float c22, float c23,
                               float c31, float c32, float c33);
    bool set_C_b_a(float c11, float c12, float c13,
                               float c21, float c22, float c23,
                               float c31, float c32, float c33);
    bool set_psi_b_a(float psi_in);
    bool set_phi_b_a(float phi_in);
    bool set_theta_b_a(float theta_in);
    bool set_euler_b_a(float phi_in, float theta_in, float psi_in);

    float get_psi_from_ref_to_euler();
    float get_phi_from_ref_to_euler();
    float get_theta_from_ref_to_euler();
    std::string get_version();

protected:
    Eigen::Matrix3f C_a_b;
    Eigen::Matrix3f C_b_a;
    float phi_b_a;
    float theta_b_a;
    float psi_b_a;

    bool update_euler_b_a();
    bool update_C();
};

#endif // ATTITUDE_H
