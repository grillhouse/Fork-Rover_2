#ifndef VECMATH_H
#define VECMATH_H

#include <cmath>

double const PI = 3.14159265359;

class vector {
public:
    double x, y;

    vector() : x(0), y(0) {} 
    vector(double tempx, double tempy) : x(tempx), y(tempy) {}

    vector operator+(const vector& a) const {
        return vector(x + a.x, y + a.y);    // vector addition
    }

    vector operator-(const vector& a) const {
        return vector(x - a.x, y - a.y);    // vector subtraction
    }

    vector operator*(double k) const {
        return vector(x * k, y * k);      // scalar multiplication
    }

    double operator*(const vector& a) const {  // dot product
        return (x * a.x + y * a.y);
    }

    double Norm() const {
        return std::sqrt(x * x + y * y);        // length of the vector
    }

    void Set(double tempx, double tempy) {  // set vector coordinates
        x = tempx;
        y = tempy;
    }
};

#endif // VECMATH_H
