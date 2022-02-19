//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) by Christian Gloor
//

#ifndef _ped_vector_h_
#define _ped_vector_h_ 1

//disable warnings on 255 char debug symbols
#pragma warning (disable : 4786)
//disable warnings on extern before template instantiation
#pragma warning (disable : 4231)

#ifdef _WIN32
#ifdef _DLL
#    define LIBEXPORT __declspec(dllexport)
#    define EXPIMP_TEMPLATE
#else
#    define LIBEXPORT __declspec(dllimport)
#    define EXPIMP_TEMPLATE extern
#endif
#else
#    define LIBEXPORT
#    define EXPIMP_TEMPLATE
#endif

#include <string>

namespace Ped {
    /// Vector helper class. This is basically a struct with some related functions attached.
    /// x, y, and z are public, so that they can be accessed easily.
    class LIBEXPORT Tvector {
    public:
        // Default constructor
        Tvector();

        // Initializing constructor
        Tvector(double px, double py, double pz = 0) : x(px), y(py), z(pz) {};

        // Methods
        double length() const;
        double lengthSquared() const;
        void normalize();
        Tvector normalized() const;
        void scale(double factor);
        Tvector scaled(double factor) const;

        Tvector leftNormalVector() const;
        Tvector rightNormalVector() const;

        double polarRadius() const;
        double polarAngle() const;

        double angleTo(const Tvector &other) const;
        void rotate(double theta);
        Ped::Tvector rotated(double theta) const;

        static double scalar(const Tvector &a, const Tvector &b);
        static double dotProduct(const Tvector &a, const Tvector &b);
        static Tvector crossProduct(const Tvector &a, const Tvector &b);

        std::string to_string() const;

        static bool lineIntersection(const Ped::Tvector &p0, const Ped::Tvector &p1, const Ped::Tvector &p2, const Ped::Tvector &p3, Ped::Tvector *intersection);

        // Operators
        Tvector operator+(const Tvector& other) const;
        Tvector operator-(const Tvector& other) const;
        Tvector operator*(double factor) const;
        Tvector operator/(double divisor) const;
        Tvector& operator+=(const Tvector& vectorIn);
        Tvector& operator-=(const Tvector& vectorIn);
        Tvector& operator*=(double factor);
        Tvector& operator*=(const Tvector& vectorIn);
        Tvector& operator/=(double divisor);

        // Attributes
        double x;
        double y;
        double z;
    };
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
Ped::Tvector operator-(const Ped::Tvector& vectorIn);
Ped::Tvector operator*(double factor, const Ped::Tvector& vector);


#endif
