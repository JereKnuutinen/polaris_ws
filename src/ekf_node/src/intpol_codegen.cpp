#include "myInterpolator2.h"
#include "intpol_codegen.h"

static MyInterpolator myInterpolatorInstance;

double int_at(double x, double y) {
    double result = myInterpolatorInstance.interpolateAt(x, y);
    return result;
}