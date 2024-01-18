

#include <iostream>
#include <vector>
#include <memory>
#include <mba.hpp>
#include <iostream>
#include <fstream>

class MyInterpolator {
public:
    MyInterpolator();
    double interpolateAt(double x, double y);
    
private:
    std::unique_ptr<mba::MBA<2>> interp;
};
