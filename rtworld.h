//
// Created by Boğaçhan Arslan on 9.11.2020.
//

#ifndef RAY_TRACING_EXAMPLE_RTWORLD_H
#define RAY_TRACING_EXAMPLE_RTWORLD_H



#include <cmath>
#include <limits>
#include <memory>


// Usings

using std::shared_ptr;
using std::make_shared;
using std::sqrt;

// Constants

const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;

// Utility Functions

inline double degrees_to_radians(double degrees) {
    return degrees * pi / 180.0;
}

// Common Headers


//Utility functions
inline double random_double() {
    // Returns a random real in [0,1).
    return rand() / (RAND_MAX + 1.0);
}

inline double random_double(double min, double max) {
    // Returns a random real in [min,max).
    return min + (max-min)*random_double();
}

class texture {
public:
    virtual glm::dvec3 value(double u, double v, const glm::dvec3& p) const = 0;
};

class solid_color : public texture {
public:
    solid_color() {}
    solid_color(glm::dvec3 c) : color_value(c) {}

    solid_color(double red, double green, double blue)
            : solid_color(glm::dvec3(red,green,blue)) {}

    virtual glm::dvec3 value(double u, double v, const glm::dvec3& p) const override {
        return color_value;
    }

private:
    glm::dvec3 color_value;
};

//Default structs

struct Ray{
    glm::dvec3 origin, direction;

    Ray(const glm::dvec3 &origin, const glm::dvec3 & direction):origin(origin),direction(direction){};
    glm::dvec3 at(double t) const{
        return origin + t*direction;
    }
};

class material;

struct hit_record {
    glm::dvec3 p;
    glm::dvec3 normal;
    shared_ptr<material> mat_ptr;
    double t;
    double u;
    double v;
    bool front_face;

    inline void set_face_normal(const Ray& r, const glm::dvec3& outward_normal) {
        front_face = glm::dot(r.direction, outward_normal) < 0;
        normal = front_face ? outward_normal :-outward_normal;
    }
};

bool near_zero(const glm::dvec3& e) {
    // Return true if the vector is close to zero in all dimensions.
    const auto s = 1e-8;
    return (fabs(e.x) < s) && (fabs(e.y) < s) && (fabs(e.z) < s);
}

class material {
public:
    virtual glm::dvec3 emitted(double u, double v, const glm::dvec3& p) const {
        return glm::dvec3(0,0,0);
    }
    virtual bool scatter(
            const Ray& r_in, const hit_record& rec, glm::dvec3& attenuation, Ray& scattered
    ) const = 0;
};



#endif //RAY_TRACING_EXAMPLE_RTWORLD_H
