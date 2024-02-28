#include "myInterpolator2.h"
#include <iostream>
#include <fstream>
//double data[36213];
//double data[23412];
double data[30261];
template<typename T> bool readFromFile(const char* filename, T* data, int size) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    for (int i = 0; i < size; ++i) {
        if (!(file >> data[i])) {
            std::cerr << "Error reading data from file: " << filename << std::endl;
            file.close();
            return false;
        }
    }

    file.close();
    return true;
}

static double evalMap(const mba::MBA<2>& interp, int size, double x, double y, int derivative_x, int derivative_y)
{
    if (derivative_x > 1 || derivative_y > 1) {
        return 0.0;
    }
    if (derivative_x == 0 && derivative_y == 0) {
        double w = interp(mba::point<2>{x, y});
        return w;
    } else if (derivative_x == 1) {
        double dx = 0.1;
        double Dx = 1/(2 * dx) * (interp(mba::point<2>{x + dx, y}) - interp(mba::point<2>{x - dx, y}));
        return Dx;
    } else if (derivative_y == 1) {
        double dy = 0.1;
        double Dy = 1/(2 * dy) * (interp(mba::point<2>{x, y + dy}) - interp(mba::point<2>{x, y - dy}));
        return Dy;
    }

    return 0.0; // Default case
}

MyInterpolator::MyInterpolator() {
    // Read Dfls_table values from a file
    //if (!readFromFile("/home/mpc/Desktop/updated_EKF/ground_profile_lla.txt", data, 7332)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
    //}
    // if (!readFromFile("/home/mpc/Desktop/ground_profile.txt", data, 7332)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
    // }
    if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/maanmitauslaitos_data/Aalto_ll_map_filtered_larger_EKF.txt", data, 30261)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
    		std::cout << " MAP reading failed in the case of EKF" << std::endl;
    }
    int size_data = sizeof(data) / sizeof(double);
    std::vector<mba::point<2>> coo;
    std::vector<double> val;
    // Initialize grid and data points
    //mba::point<2> lo = {354439.9500200000, 6704369.980090001};
    //mba::point<2> hi = {354500.0499800000, 6704419.989910000};
    //mba::point<2> lo = {24.353891932679812-0.00000000001, 60.449254646676444-0.00000000001};
    //mba::point<2> hi = {24.355007179789787+0.00000000001,  60.449721900186482+0.00000000001};
    // For otaniemi
    //mba::point<2> lo = {24.818043915382614-0.00000000001, 60.187121967051709-0.00000000001};
    //mba::point<2> hi = {24.821661539730453+0.00000000001,  60.188072906353234+0.00000000001}; 
    mba::point<2> lo = {24.816579309883824-0.00000000001, 60.187098785392173-0.00000000001};
    mba::point<2> hi = {24.821128063840732+0.00000000001,  60.188063892751543+0.00000000001}; 
    mba::index<2> grid = {2, 2};
    for (int i = 0; i < size_data; i += 3) {
        mba::point<2> point;
        point[0] = data[i];
        point[1] = data[i + 1];
        coo.push_back(point);
        double value = data[i + 2];
        val.push_back(value);
    }
    interp = std::unique_ptr<mba::MBA<2>>(new mba::MBA<2>(lo, hi, grid, coo, val));
    std::cout << "meni interp:sta" << std::endl;
}

double MyInterpolator::interpolateAt(double x, double y) {
    return evalMap(*interp, sizeof(data) / sizeof(double), x, y, 0, 0);
}

// Declare a static member variable to store the MyInterpolator instance
// static MyInterpolator myInterpolatorInstance;
// 
// double int_at(double x, double y) {
// 
// 
//     double result = myInterpolatorInstance.interpolateAt(x, y);
//     return results;
// }
// void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
//     if (nrhs != 2) {
//         mexErrMsgTxt("Two input arguments (x and y) are required.");
//         return;
//     }
// 
//     double x = mxGetScalar(prhs[0]);
//     double y = mxGetScalar(prhs[1]);
// 
//     double result = myInterpolatorInstance.interpolateAt(x, y);
// 
//     plhs[0] = mxCreateDoubleScalar(result);
// }


