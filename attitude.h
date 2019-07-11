#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "Eigen/Core"

#define ATTITUDE_VERSION    "Attitude v.0.1.0.0"

class Attitude
{
public:
    Eigen::Matrix3f DCM;
    float phi;
    float theta;
    float psi;

    Attitude();

    bool set_DCM(float c11, float c12, float c13,
                 float c21, float c22, float c23,
                 float c31, float c32, float c33);
    bool set_DCM(Eigen::Matrix3f DCM_in);

    bool set_psi(float psi_in);
    bool set_phi(float phi_in);
    bool set_theta(float theta_in);
    bool set_euler(float phi_in, float theta_in, float psi_in);

    float get_psi();
    float get_phi();
    float get_theta();
    Eigen::Matrix3f get_DCM();


    bool update_euler();
    bool update_C();

    std::string get_version();

protected:



};

#endif // ATTITUDE_H
