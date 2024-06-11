#include "myInterpolator2.h"
#include <iostream>
#include <fstream>
//double data[36213];
//double data[23412];
//double data[30261];
//double data[7332];
//double data[38307];
double data[38307];
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

    return 0.0;
}

MyInterpolator::MyInterpolator() {
    // Read Dfls_table values from a file
    //if (!readFromFile("/home/mpc/Desktop/updated_EKF/ground_profile_lla.txt", data, 7332)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
   // }
    // if (!readFromFile("/home/mpc/Desktop/ground_profile.txt", data, 7332)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
    // }
    //if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/maanmitauslaitos_data/Aalto_ll_map_filtered_larger_EKF.txt", data, 30261)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
    //		std::cout << " MAP reading failed in the case of EKF" << std::endl;
    //}
    //if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/vakola_concrete_data_maaliskuu/vakola_ll_map_maaliskuu_rear_v4_EKF.txt", data, 43200)) {
    //     //mexErrMsgTxt("Failed to read data from file.");
    //		std::cout << " MAP reading failed in the case of EKF" << std::endl;
   // }
    if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/Vakola_concrete_data_toukokuu_AOD/vakola_ll_map_toukokuu_AOD_EKF.txt", data, 38307)) {
         //mexErrMsgTxt("Failed to read data from file.");
    		std::cout << " MAP reading failed in the case of EKF" << std::endl;
   }
   //if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/Forest_data_toukokuu_UAV_path1/forest_ll_map_toukokuu_UAV_EKF.txt", data, 42966)) {
         //mexErrMsgTxt("Failed to read data from file.");
    	//	std::cout << " MAP reading failed in the case of EKF" << std::endl;
  // }
   //if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/Forest_data_toukokuu_UAV_path2/forest_ll_map_toukokuu_UAV_path2_EKF.txt", data, 90801)) {
         //mexErrMsgTxt("Failed to read data from file.");
    		//std::cout << " MAP reading failed in the case of EKF" << std::endl;
  // }
     // if (!readFromFile("/home/mpc/viatoc_matlab/rollover_nmpc-main/Forest_data_toukokuu_UAV_path3/forest_ll_map_toukokuu_UAV_path3_EKF.txt", data, 68670)) {
         //mexErrMsgTxt("Failed to read data from file.");
    		//std::cout << " MAP reading failed in the case of EKF" << std::endl;
  // }
    int size_data = sizeof(data) / sizeof(double);
    std::vector<mba::point<2>> coo;
    std::vector<double> val;
    // Initialize grid and data points
    //mba::point<2> lo = {354439.9500200000, 6704369.980090001};
    //mba::point<2> hi = {354500.0499800000, 6704419.989910000};
    
    // Vakola NLS map
    //mba::point<2> lo = {24.353891932679812-0.00000000001, 60.449254646676444-0.00000000001};
    //mba::point<2> hi = {24.355007179789787+0.00000000001,  60.449721900186482+0.00000000001};
    // For otaniemi
    //mba::point<2> lo = {24.818043915382614-0.00000000001, 60.187121967051709-0.00000000001};
    //mba::point<2> hi = {24.821661539730453+0.00000000001,  60.188072906353234+0.00000000001}; 
    // For larger otaniemi
    //mba::point<2> lo = {24.816579309883824-0.00000000001, 60.187098785392173-0.00000000001};
    //mba::point<2> hi = {24.821128063840732+0.00000000001,  60.188063892751543+0.00000000001}; 
    // For vakola concrete
    //mba::point<2> lo = {24.354154851175654-0.00000000001, 60.449316778362487-0.00000000001};
    //mba::point<2> hi = {24.354829075732617+0.00000000001,  60.449649844775443+0.00000000001}; 
    
    // Vakola new AOD map
    mba::point<2> lo = {24.354174850537071-0.00000000001, 60.449326110281270-0.00000000001};
    mba::point<2> hi = {24.354809414815332+0.00000000001,  60.449639584551527+0.00000000001};
    // Forest path 1 UAV map
    //mba::point<2> lo = {24.347244777327845-0.00000000001, 60.451263307777126-0.00000000001};
    //mba::point<2> hi = {24.348002436842062+0.00000000001,  60.452106677875882+0.00000000001};
    
    // Forest path 2 UAV map
    //mba::point<2> lo = {24.347758592990875-0.00000000001, 60.452046595974068-0.00000000001}; // Num of ponts in path2: 90801
   // mba::point<2> hi = {24.349418641286540+0.00000000001,  60.452840760472291+0.00000000001};
    
    // Forest path 3 UAV map
    //mba::point<2> lo = {24.349515873611765-0.00000000001, 60.452842260802576-0.00000000001}; // Num of points in path3: 68670
    //mba::point<2> hi = {24.350572815550880+0.00000000001,  60.453799026626044+0.00000000001};  
      
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
    std::cout << "meni interpsta ekf" << std::endl;
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


