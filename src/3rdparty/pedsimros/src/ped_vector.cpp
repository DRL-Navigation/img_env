//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor
//

#include "ped_vector.h"

#include <cmath>
#include <string>

#ifdef _WIN32
const double M_PI = 3.14159265359;
#endif


/// Default constructor, which makes sure that all the values are set to 0.
/// \date    2012-01-16
Ped::Tvector::Tvector() : x(0), y(0), z(0) {};


std::string Ped::Tvector::to_string() const {
    return std::to_string(x) + "/" + std::to_string(y) + "/" + std::to_string(z);
}


/// Returns the length of the vector.
/// \return the length
double Ped::Tvector::length() const {
    if ((x == 0) && (y == 0) && (z == 0)) return 0;
    return sqrt(lengthSquared());
}


/// Returns the length of the vector squared. This is faster than the real length.
/// \return the length squared
double Ped::Tvector::lengthSquared() const {
    return x*x + y*y + z*z;
}


/// Normalizes the vector to a length of 1.
/// \date    2010-02-12
void Ped::Tvector::normalize() {
    double len = length();

    // null vectors cannot be normalized
    if (len == 0) return;

    x /= len;
    y /= len;
    z /= len;
}


/// Normalizes the vector to a length of 1.
/// \date    2013-08-02
Ped::Tvector Ped::Tvector::normalized() const {
    double len = length();

    // null vectors cannot be normalized
    if (len == 0) return Ped::Tvector();;

    return Ped::Tvector(x/len, y/len, z/len);
}


/// Vector scalar product helper: calculates the scalar product of two vectors.
/// \date    2012-01-14
/// \return  The scalar product.
/// \param   &a The first vector
/// \param   &b The second vector
double Ped::Tvector::scalar(const Ped::Tvector &a, const Ped::Tvector &b) {
    return acos( Ped::Tvector::dotProduct(a, b) / ( a.length() * b.length() ) );
}


/// Vector dot product helper: calculates the dot product of two vectors.
/// \date    2012-01-14
/// \return  The dot product.
/// \param   &a The first vector
/// \param   &b The second vector
double Ped::Tvector::dotProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
    return (a.x*b.x + a.y*b.y + a.z*b.z);
}


/// Calculates the cross product of two vectors.
/// \date    2010-02-12
/// \param   &a The first vector
/// \param   &b The second vector
Ped::Tvector Ped::Tvector::crossProduct(const Ped::Tvector &a, const Ped::Tvector &b) {
    return Ped::Tvector(
        a.y*b.z - a.z*b.y,
        a.z*b.x - a.x*b.z,
        a.x*b.y - a.y*b.x);
}


/// Scales this vector by a given factor in each dimension.
/// \date    2013-08-02
/// \param   factor The scalar value to multiply with.
void Ped::Tvector::scale(double factor) {
    x *= factor;
    y *= factor;
    z *= factor;
}


/// Returns a copy of this vector which is multiplied in each dimension by a given factor.
/// \date    2013-07-16
/// \return  The scaled vector.
/// \param   factor The scalar value to multiply with.
Ped::Tvector Ped::Tvector::scaled(double factor) const {
    return Ped::Tvector(factor*x, factor*y, factor*z);
}

// \warning: This is in 2D only!
Ped::Tvector Ped::Tvector::leftNormalVector() const {
    return Ped::Tvector(-y, x);
}


// \warning: This is in 2D only!
Ped::Tvector Ped::Tvector::rightNormalVector() const {
    return Ped::Tvector(y, -x);
}


double Ped::Tvector::polarRadius() const {
    return length();
}


// \warning: This is in 2D only!
double Ped::Tvector::polarAngle() const {
    return atan2(y, x);
}


// \warning: This is in 2D only!
double Ped::Tvector::angleTo(const Tvector &other) const {
    double angleThis = polarAngle();
    double angleOther = other.polarAngle();

    // compute angle
    double diffAngle = angleOther - angleThis;
    // → normalize angle
    if (diffAngle > M_PI) diffAngle -= 2 * M_PI;
    else if(diffAngle <= -M_PI) diffAngle += 2 * M_PI;

    return diffAngle;
}


Ped::Tvector Ped::Tvector::operator+(const Tvector& other) const {
    return Ped::Tvector(
        x + other.x,
        y + other.y,
        z + other.z);
}


Ped::Tvector Ped::Tvector::operator-(const Tvector& other) const {
    return Ped::Tvector(
        x - other.x,
        y - other.y,
        z - other.z);
}


Ped::Tvector Ped::Tvector::operator*(double factor) const {
    return scaled(factor);
}


Ped::Tvector Ped::Tvector::operator/(double divisor) const {
    return scaled(1/divisor);
}


Ped::Tvector& Ped::Tvector::operator+=(const Tvector& vectorIn) {
    x += vectorIn.x;
    y += vectorIn.y;
    z += vectorIn.z;
    return *this;
}


Ped::Tvector& Ped::Tvector::operator-=(const Tvector& vectorIn) {
    x -= vectorIn.x;
    y -= vectorIn.y;
    z -= vectorIn.z;
    return *this;
}


Ped::Tvector& Ped::Tvector::operator*=(double factor) {
    scale(factor);
    return *this;
}


Ped::Tvector& Ped::Tvector::operator*=(const Tvector& vectorIn) {
    x *= vectorIn.x;
    y *= vectorIn.y;
    z *= vectorIn.z;
    return *this;
}


Ped::Tvector& Ped::Tvector::operator/=(double divisor) {
    scale(1/divisor);
    return *this;
}


bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return (vector1In.x == vector2In.x)
        && (vector1In.y == vector2In.y)
        && (vector1In.z == vector2In.z);
}


bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return (vector1In.x != vector2In.x)
        || (vector1In.y != vector2In.y)
        || (vector1In.z != vector2In.z);
}


Ped::Tvector operator+(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return Ped::Tvector(
        vector1In.x + vector2In.x,
        vector1In.y + vector2In.y,
        vector1In.z + vector2In.z);
}


Ped::Tvector operator-(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In) {
    return Ped::Tvector(
        vector1In.x - vector2In.x,
        vector1In.y - vector2In.y,
        vector1In.z - vector2In.z);
}


Ped::Tvector operator-(const Ped::Tvector& vectorIn) {
    return Ped::Tvector(
        -vectorIn.x,
        -vectorIn.y,
        -vectorIn.z);
}


Ped::Tvector operator*(double factor, const Ped::Tvector& vector) {
    return vector.scaled(factor);
}


/// Calculates the itnersection point of two lines, defined by Ped::Tvectors p0, p1, and p2, p3 respectively.
/// Based on an algorithm in Andre LeMothe's "Tricks of the Windows Game Programming Gurus"
/// \return bool True if there is an intersection, false otherwise
/// \return *intersection If the supplied pointer to a Ped::Tvector is not NULL, it will contain the intersection point, if there is an intersection.
bool Ped::Tvector::lineIntersection(const Ped::Tvector &p0, const Ped::Tvector &p1, const Ped::Tvector &p2, const Ped::Tvector &p3, Ped::Tvector *intersection) {
  Ped::Tvector s1(p1.x - p0.x, p1.y - p0.y);
  Ped::Tvector s2(p3.x - p2.x, p3.y - p2.y);

  double s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / (-s2.x * s1.y + s1.x * s2.y);
  double t = ( s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / (-s2.x * s1.y + s1.x * s2.y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) { // intersection
    if (intersection != NULL) {
      intersection->x = p0.x + (t * s1.x);
      intersection->y = p0.y + (t * s1.y);
    }
    return true;
  }

  return false; // No intersection
}

/// Rotates a vector. 
/// Rotates around 0,0 in 2 dimensions only (z unchanged)v
/// \param theta in rad
void Ped::Tvector::rotate(double theta) { // theta in rad. 
  double xt = x * cos(theta) - y * sin(theta);
  double yt = x * sin(theta) + y * cos(theta);
  x = xt; y = yt;
}

/// Rotates a vector.
/// Rotates around 0,0 in 2 dimensions only (z set to 0.0)
/// \param theta in rad
Ped::Tvector Ped::Tvector::rotated(double theta) const { // theta in rad
  return Ped::Tvector(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta)); // let's hope the compiler reuses sin/cos
}

