#include "attitude.h"
#include <cmath>

Attitude::Attitude()
{

}


bool Attitude::set_DCM(float c11, float c12, float c13,
                       float c21, float c22, float c23,
                       float c31, float c32, float c33)
{
    DCM <<  c11, c12, c13,
            c21, c22, c23,
            c31, c32, c33;

    return true;
}

bool Attitude::set_DCM(Eigen::Matrix3f DCM_in)
{
    DCM = DCM_in;
    return true;
}


bool Attitude::set_psi(float psi_in)
{
    psi = psi_in;
    update_C();
    return true;
}

bool Attitude::set_phi(float phi_in)
{
    phi = phi_in;
    return true;
}

bool Attitude::set_theta(float theta_in)
{
    theta = theta_in;
    return true;
}

bool Attitude::set_euler(float phi_in, float theta_in, float psi_in)
{
    phi = phi_in;
    theta = theta_in;
    psi = psi_in;
    return true;
}

float Attitude::get_psi()
{
    return psi;
}

float Attitude::get_phi()
{
    return phi;
}

float Attitude::get_theta()
{
    return theta;
}

Eigen::Matrix3f Attitude::get_DCM()
{
    return DCM;
}

std::string Attitude::get_version()
{
    return ATTITUDE_VERSION;
}

bool Attitude::update_euler()
{
    phi     = atan2f(DCM(2,1), DCM(2,2));
    theta   = -asinf(DCM(2,0));
    psi     = atan2f(DCM(1,0), DCM(0,0));

    return true;
}

bool Attitude::update_C()
{
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float cos_phi   = cosf(phi);
    float sin_phi   = sinf(phi);
    float cos_psi   = cosf(psi);
    float sin_psi   = cosf(psi);

    DCM(0,0) = cos_theta * cos_psi;
    DCM(0,1) = -cos_phi * sin_psi +
               sin_phi * sin_theta * cos_psi;
    DCM(0,2) = sin_phi * sin_psi +
               cos_phi * sin_theta * cos_psi;

    DCM(1,0) = cos_theta * sin_psi;
    DCM(1,1) = cos_phi * cos_psi +
               sin_phi * sin_theta * sin_psi;
    DCM(1,2) = -sin_phi * cos_psi +
               cos_phi * sin_theta * sin_psi;

    DCM(2,0) = -sin_theta;
    DCM(2,1) = sin_phi * cos_theta;
    DCM(2,2) = cos_phi * cos_theta;

    return true;
}








