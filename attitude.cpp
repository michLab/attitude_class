#include "attitude.h"
#include <cmath>

Attitude::Attitude()
{

}

bool Attitude::set_C_from_obj_to_ref(float c11, float c12, float c13,
                                     float c21, float c22, float c23,
                                     float c31, float c32, float c33)
{
    C_from_obj_to_ref <<    c11, c12, c13,
                            c21, c22, c23,
                            c31, c32, c33;

    C_from_ref_to_obj = C_from_obj_to_ref.transpose();

    update_euler_from_ref_to_obj();
    return true;
}

bool Attitude::set_C_from_ref_to_obj(float c11, float c12, float c13, float c21, float c22, float c23, float c31, float c32, float c33)
{
    C_from_ref_to_obj <<    c11, c12, c13,
                            c21, c22, c23,
                            c31, c32, c33;

    C_from_obj_to_ref = C_from_ref_to_obj.transpose();

    update_euler_from_ref_to_obj();
    return true;
}

bool Attitude::set_psi_from_ref_to_obj(float psi_in)
{
    psi_from_ref_to_obj = psi_in;
    update_C();
    return true;
}

bool Attitude::set_phi_from_ref_to_obj(float phi_in)
{
    phi_from_ref_to_obj = phi_in;
    update_C();
    return true;
}

bool Attitude::set_theta_from_ref_to_obj(float theta_in)
{
    theta_from_ref_to_obj = theta_in;
    update_C();
    return true;
}

bool Attitude::set_euler_from_ref_to_obj(float phi_in, float theta_in, float psi_in)
{
    phi_from_ref_to_obj = phi_in;
    theta_from_ref_to_obj = theta_in;
    psi_from_ref_to_obj = psi_in;
    update_C();
    return true;
}

float Attitude::get_psi_from_ref_to_euler()
{
    return psi_from_ref_to_obj;
}

float Attitude::get_phi_from_ref_to_euler()
{
    return phi_from_ref_to_obj;
}

float Attitude::get_theta_from_ref_to_euler()
{
    return theta_from_ref_to_obj;
}

bool Attitude::update_euler_from_ref_to_obj()
{
    phi_from_ref_to_obj = atan2f(C_from_obj_to_ref(2,1),
                                 C_from_obj_to_ref(2,2));
    theta_from_ref_to_obj = -asinf(C_from_obj_to_ref(2,0));
    psi_from_ref_to_obj = atan2f(C_from_obj_to_ref(1,0),
                                 C_from_obj_to_ref(0,0));

    return true;
}

bool Attitude::update_C()
{
    float cos_theta = cosf(theta_from_ref_to_obj);
    float sin_theta = sinf(theta_from_ref_to_obj);
    float cos_phi = cosf(phi_from_ref_to_obj);
    float sin_phi = sinf(phi_from_ref_to_obj);
    float cos_psi = cosf(psi_from_ref_to_obj);
    float sin_psi = cosf(psi_from_ref_to_obj);

    C_from_obj_to_ref(0,0) = cos_theta * cos_psi;
    C_from_obj_to_ref(0,1) = -cos_phi * sin_psi +
                              sin_phi * sin_theta * cos_psi;
    C_from_obj_to_ref(0,2) = sin_phi * sin_psi +
                             cos_phi * sin_theta * cos_psi;

    C_from_obj_to_ref(1,0) = cos_theta * sin_psi;
    C_from_obj_to_ref(1,1) = cos_phi * cos_psi +
                             sin_phi * sin_theta * sin_psi;
    C_from_obj_to_ref(1,2) = -sin_phi * cos_psi +
                             cos_phi * sin_theta * sin_psi;

    C_from_obj_to_ref(2,0) = -sin_theta;
    C_from_obj_to_ref(2,1) = sin_phi * cos_theta;
    C_from_obj_to_ref(2,2) = cos_phi * cos_theta;

    C_from_ref_to_obj = C_from_obj_to_ref.transpose();

    return true;
}








