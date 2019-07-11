#include <iostream>
#include "attitude.h"

using namespace std;

int main()
{
    Attitude a, b, c;
    std::cout << a.get_version() << std::endl;
    a.set_euler(0.1f, 0.2f, 0.3f);
    b.set_euler(0.1f, 0.2f, 0.3f);
    c.set_DCM(a.get_DCM() * b.get_DCM());
    std::cout << a.get_DCM() << std::endl;
    std::cout << std::endl;
    std::cout << b.get_DCM() << std::endl;
    std::cout << std::endl;
    std::cout << c.get_DCM() << std::endl;
    return 0;
}
