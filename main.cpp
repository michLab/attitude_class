#include <iostream>
#include "attitude.h"

using namespace std;

int main()
{
    Attitude att;
    att.set_euler_from_ref_to_obj(0.1f, 0.2f, 0.3f);
    return 0;
}
