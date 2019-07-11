#include <iostream>
#include "attitude.h"

using namespace std;

int main()
{
    Attitude att;
    std::cout << att.get_version() << std::endl;
    att.set_euler_b_a(0.1f, 0.2f, 0.3f);
    return 0;
}
