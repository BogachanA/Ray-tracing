#include <iostream>
#include <vector>
#include "IO.h"
#include "PixelBuffer.h"
#include "rtworld.h"
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

glm::dvec3 random_dvec3(double min, double max){
    return glm::dvec3(random_double(min,max), random_double(min,max), random_double(min,max));
}

glm::dvec3 random_in_unit_sphere() {
    while (true) {
        auto p = random_dvec3(-1,1);
        if (glm::dot(p,p) >= 1) continue;
        return p;
    }
}

glm::dvec3 random_unit_vector(){
    return glm::normalize(random_in_unit_sphere());
}

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
    virtual bool scatter(
            const Ray& r_in, const hit_record& rec, glm::dvec3& attenuation, Ray& scattered
    ) const = 0;
};

//Lambertian reflection model
class lambertian : public material {
public:
    lambertian(const glm::dvec3& a) : albedo(a) {}

    virtual bool scatter(
            const Ray& r_in, const hit_record& rec, glm::dvec3& attenuation, Ray& scattered
    ) const override {
        auto scatter_direction = rec.normal + random_unit_vector();
        if (near_zero(scatter_direction))
            scatter_direction = rec.normal;
        scattered = Ray(rec.p, scatter_direction);
        attenuation = albedo;
        return true;
    }

public:
    glm::dvec3 albedo;
};

//Metal material
glm::dvec3 reflect(const glm::dvec3& v, const glm::dvec3& n) {
    return v - 2*glm::dot(v,n)*n;
}

class metal : public material {
public:
    metal(const glm::dvec3& a) : albedo(a) {}

    bool scatter(
            const Ray& r_in, const hit_record& rec, glm::dvec3& attenuation, Ray& scattered
    ) const override {
        glm::dvec3 reflected = reflect(glm::normalize(r_in.direction), rec.normal);
        scattered = Ray(rec.p, reflected);
        attenuation = albedo;
        return (glm::dot(scattered.direction, rec.normal) > 0);
    }

public:
    glm::dvec3 albedo;
};




class hittable {
public:
    virtual bool hit(const Ray& r, double t_min, double t_max, hit_record& rec) const = 0;
};

class Sphere : public hittable {
public:
    Sphere() {}
    Sphere(glm::dvec3 cen, double r, shared_ptr<material> m) : center(cen), radius(r), mat_ptr(m) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, hit_record& rec) const override {
        glm::dvec3 oc = r.origin - center;
        auto a = glm::dot(r.direction, r.direction);
        auto half_b = glm::dot(oc, r.direction);
        auto c = glm::dot(oc, oc) - radius*radius;

        auto discriminant = half_b*half_b - a*c;
        if (discriminant < 0) return false;
        auto sqrtd = sqrt(discriminant);

        // Find the nearest root that lies in the acceptable range.
        auto root = (-half_b - sqrtd) / a;
        if (root < t_min || t_max < root) {
            root = (-half_b + sqrtd) / a;
            if (root < t_min || t_max < root)
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        glm::dvec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat_ptr = mat_ptr;

        return true;
    }

public:
    glm::dvec3 center;
    double radius;
    shared_ptr<material> mat_ptr;
};

class hittable_list : public hittable {
public:
    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(shared_ptr<hittable> object) { objects.push_back(object); }

    virtual bool hit(
            const Ray& r, double t_min, double t_max, hit_record& rec) const override;

public:
    std::vector<shared_ptr<hittable>> objects;
};

bool hittable_list::hit(const Ray& r, double t_min, double t_max, hit_record& rec) const {
    hit_record temp_rec;
    bool hit_anything = false;
    auto closest_so_far = t_max;

    for (const auto& object : objects) {
        if (object->hit(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}




glm::dvec3 ray_color(const Ray& r, const hittable& world, int depth) {
    hit_record rec;

    if (depth <= 0)
        return glm::dvec3(0,0,0);

    if (world.hit(r, 0.01, infinity, rec)) {
        Ray scattered(glm::dvec3(0),random_unit_vector());
        glm::dvec3 attenuation;
        if (rec.mat_ptr->scatter(r, rec, attenuation, scattered))
            return attenuation * ray_color(scattered, world, depth-1);
        return glm::dvec3(0,0,0);
    }
    glm::dvec3 unit_direction = glm::normalize(r.direction);
    auto t = 0.5*(unit_direction.y + 1.0);
    return (1.0-t)*glm::dvec3(1.0, 1.0, 1.0) + t*glm::dvec3(0.5, 0.7, 1.0);
}

int main() {

    // World
    hittable_list world;
    auto material_ground = make_shared<lambertian>(glm::dvec3(0.8, 0.8, 0.0));
    auto material_center = make_shared<lambertian>(glm::dvec3(0.9725, 0.6314, 0.902));
    auto material_left   = make_shared<metal>(glm::dvec3(0.9725, 0.6314, 0.902));
    auto material_right  = make_shared<metal>(glm::dvec3(0.9725, 0.6314, 0.902));
    auto yellow_lamb = make_shared<metal>(glm::dvec3(0.8941,0.6078,0.0588));

    world.add(make_shared<Sphere>(glm::dvec3( 0.0, -100.5, -1.0), 100.0, material_ground));
    world.add(make_shared<Sphere>(glm::dvec3( 0.0,    0.0, 0.0),   0.5, material_center));
    world.add(make_shared<Sphere>(glm::dvec3(-1.3,    0.70, -1.0),   0.5, material_left));
    world.add(make_shared<Sphere>(glm::dvec3( 1.0,    0.0, 1.0),   0.5, material_right));
    world.add(make_shared<Sphere>(glm::dvec3( 0.6,    1.12, -1.0),   0.5, yellow_lamb));

    const int max_depth = 20;

    PixelBuffer pixel_buffer(glm::ivec2(640,480));
    Camera cam(glm::dvec3(0,0,3), glm::dvec3(0,0,0),pixel_buffer);

    for (int y = 0; y < pixel_buffer.dimensions.y; ++y) {
        for (int x = 0; x < pixel_buffer.dimensions.x; ++x) {
            glm::dvec3 color(0);

            for (int i = 0; i < samples_per_pixel; ++i) {
                auto u = (x + random_double()) / (pixel_buffer.dimensions.x-1);
                auto v = (y + random_double()) / (pixel_buffer.dimensions.y-1);
                glm::dvec2 uv(u,v);
                Ray ray(cam.position, glm::normalize(cam.raster_to_world(uv) - cam.position));
                color+=ray_color(ray,world,max_depth);
            }


            /*
            bool hit_any=false;
            for (Sphere &sphere: spheres){
                if(sphere.hit(ray)!=-1.0) {
                    hit_any=true;
                    color = sphere.ray_color(ray);
                }
            }
            */


            color/=samples_per_pixel;
            pixel_buffer.set(x,pixel_buffer.dimensions.y - y -1,color);
        }
    }

    ofstream output;
    output.open("./task4.ppm",ios::out | ios::trunc);
    if(!output.is_open()) return 1;

    cout << "Outputting...";
    IO::write_as_PPM(pixel_buffer,output);
    output.close();
    cout << "done!" << endl;
    return 0;
}