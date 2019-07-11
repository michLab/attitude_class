#include "attitude.h"
#include <cmath>

Attitude::Attitude()
{

}

bool Attitude::set_C_a_b(float c11, float c12, float c13,
                                     float c21, float c22, float c23,
                                     float c31, float c32, float c33)
{
    C_a_b <<    c11, c12, c13,
                            c21, c22, c23,
                            c31, c32, c33;

    C_b_a = C_a_b.transpose();

    update_euler_b_a();
    return true;
}

bool Attitude::set_C_b_a(float c11, float c12, float c13, float c21, float c22, float c23, float c31, float c32, float c33)
{
    C_b_a <<    c11, c12, c13,
                            c21, c22, c23,
                            c31, c32, c33;

    C_a_b = C_b_a.transpose();

    update_euler_b_a();
    return true;
}

bool Attitude::set_psi_b_a(float psi_in)
{
    psi_b_a = psi_in;
    update_C();
    return true;
}

bool Attitude::set_phi_b_a(float phi_in)
{
    phi_b_a = phi_in;
    update_C();
    return true;
}

bool Attitude::set_theta_b_a(float theta_in)
{
    theta_b_a = theta_in;
    update_C();
    return true;
}

bool Attitude::set_euler_b_a(float phi_in, float theta_in, float psi_in)
{
    phi_b_a = phi_in;
    theta_b_a = theta_in;
    psi_b_a = psi_in;
    update_C();
    return true;
}

float Attitude::get_psi_from_ref_to_euler()
{
    return psi_b_a;
}

float Attitude::get_phi_from_ref_to_euler()
{
    return phi_b_a;
}

float Attitude::get_theta_from_ref_to_euler()
{
    return theta_b_a;
}

bool Attitude::update_euler_b_a()
{
    phi_b_a = atan2f(C_a_b(2,1), C_a_b(2,2));
    theta_b_a = -asinf(C_a_b(2,0));
    psi_b_a = atan2f(C_a_b(1,0), C_a_b(0,0));

    return true;
}

bool Attitude::update_C()
{
    float cos_theta = cosf(theta_b_a);
    float sin_theta = sinf(theta_b_a);
    float cos_phi = cosf(phi_b_a);
    float sin_phi = sinf(phi_b_a);
    float cos_psi = cosf(psi_b_a);
    float sin_psi = cosf(psi_b_a);

    C_a_b(0,0) = cos_theta * cos_psi;
    C_a_b(0,1) = -cos_phi * sin_psi +
                              sin_phi * sin_theta * cos_psi;
    C_a_b(0,2) = sin_phi * sin_psi +
                             cos_phi * sin_theta * cos_psi;

    C_a_b(1,0) = cos_theta * sin_psi;
    C_a_b(1,1) = cos_phi * cos_psi +
                             sin_phi * sin_theta * sin_psi;
    C_a_b(1,2) = -sin_phi * cos_psi +
                             cos_phi * sin_theta * sin_psi;

    C_a_b(2,0) = -sin_theta;
    C_a_b(2,1) = sin_phi * cos_theta;
    C_a_b(2,2) = cos_phi * cos_theta;

    C_b_a = C_a_b.transpose();

    return true;
}








