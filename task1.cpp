#include <iostream>
#include <vector>
#include "glm/glm.hpp"
#include "IO.h"
#include "PixelBuffer.h"
using namespace std;

struct Camera{
    glm::dvec3 position, left_bottom,horizontal,vertical;
    double focal_length;

    Camera(const glm::dvec3 position, const glm::dvec3 target, const PixelBuffer& pixel_buffer, double focal_length=1.0)
    :position(position),focal_length(focal_length){
        glm::dvec3 up(0,1,0);
        auto forward = glm::normalize(target-position);
        auto right = glm::normalize(glm::cross(forward,up));

        up=glm::cross(forward,-right);

        horizontal = right*(double(pixel_buffer.dimensions.x) / pixel_buffer.dimensions.y);
        vertical = up*1.0;
        left_bottom = position + forward * focal_length - horizontal * 0.5 - vertical * 0.5;
    };

    glm::dvec3 raster_to_world(const glm::dvec2& r){
        return left_bottom + r.x * horizontal + r.y*vertical;
    }

};

struct Ray{
    glm::dvec3 origin, direction;

    Ray(const glm::dvec3 &origin, const glm::dvec3 & direction):origin(origin),direction(direction){};
    glm::dvec3 at(double t) const{
        return origin + t*direction;
    }
};

struct Sphere{
    glm::dvec3 center;
    double radius;

    Sphere(glm::dvec3 o, double r):center(o),radius(r){};
    double hit(const Ray& ray) const{
        glm::dvec3 oc = ray.origin - center;
        auto a = glm::dot(ray.direction, ray.direction);
        auto b = 2.0 * glm::dot(oc, ray.direction);
        auto c = glm::dot(oc, oc) - radius*radius;
        auto discriminant = b*b - 4*a*c;
        if (discriminant < 0) {
            return -1.0;
        } else {
            return (-b - glm::sqrt(discriminant) ) / (2.0*a);
        }
    }

    glm::dvec3 ray_color(const Ray& r) {
        auto t = hit(r);
        if (t > 0.0) {
            glm::dvec3 N = glm::normalize(r.at(t) - glm::dvec3(0,0,-1));
            return 0.5*glm::dvec3(N.x+1, N.y+1, N.z+1);
        }
        return glm::dvec3(0);
        /* Standart background colors
        glm::dvec3 unit_direction = glm::normalize(r.direction);
        t = 0.5*(unit_direction.y + 1.0);
        return (1.0-t)*glm::dvec3(1.0, 1.0, 1.0) + t*glm::dvec3(0.5, 0.7, 1.0);
         */
    }
};

glm::dvec3 std_back(const Ray& r){
    glm::dvec3 unit_direction = glm::normalize(r.direction);
    auto t = 0.5*(unit_direction.y + 1.0);
    return (1.0-t)*glm::dvec3(1.0, 1.0, 1.0) + t*glm::dvec3(0.5, 0.7, 1.0);
}

int main() {

    PixelBuffer pixel_buffer(glm::ivec2(640,480));
    Camera cam(glm::dvec3(2,3,6), glm::dvec3(0,0,0),pixel_buffer);

    vector<Sphere> spheres{
            {glm::dvec3(-1,3,-2), 0.4},
            {glm::dvec3(2.5,2.8,-2), 1.2},
            {glm::dvec3(-3,0,-1), 1.2},
            {glm::dvec3(0,0,0), 0.7},
            {glm::dvec3(2,-2,1), 1},
    };
    for (int y = 0; y < pixel_buffer.dimensions.y; ++y) {
        for (int x = 0; x < pixel_buffer.dimensions.x; ++x) {
            glm::dvec3 color(0);

            glm::dvec2 uv(x,y);
            uv/=pixel_buffer.dimensions;
            Ray ray(cam.position, glm::normalize(cam.raster_to_world(uv) - cam.position));

            bool hit_any=false;
            for (Sphere &sphere: spheres){
                if(sphere.hit(ray)!=-1.0) {
                    hit_any=true;
                    color = sphere.ray_color(ray);
                }
            }

            if(!hit_any){
                color=std_back(ray);
            }

            pixel_buffer.set(x,pixel_buffer.dimensions.y - y -1,color);
        }
    }

    ofstream output;
    output.open("./task1_cam2.ppm",ios::out | ios::trunc);
    if(!output.is_open()) return 1;

    cout << "Outputting...";
    IO::write_as_PPM(pixel_buffer,output);
    output.close();
    cout << "done!" << endl;
    return 0;
}