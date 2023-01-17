#include <iostream>
#include <vector>
#include "IO.h"
#include "PixelBuffer.h"
#include "rtworld.h"
#include "aabb.h"
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


//Light emitting material
class diffuse_light : public material  {
public:
    diffuse_light(shared_ptr<texture> a) : emit(a) {}
    diffuse_light(glm::dvec3 c) : emit(make_shared<solid_color>(c)) {}

    virtual bool scatter(
            const Ray& r_in, const hit_record& rec, glm::dvec3& attenuation, Ray& scattered
    ) const override {
        return false;
    }

    virtual glm::dvec3 emitted(double u, double v, const glm::dvec3& p) const override {
        return emit->value(u, v, p);
    }

public:
    shared_ptr<texture> emit;
};



class hittable {
public:
    virtual bool hit(const Ray& r, double t_min, double t_max, hit_record& rec) const = 0;
    virtual bool bounding_box(double time0, double time1, aabb& output_box) const = 0;
};

class Sphere : public hittable {
public:
    Sphere() {}
    Sphere(glm::dvec3 cen, double r, shared_ptr<material> m) : center(cen), radius(r), mat_ptr(m) {};

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override{
        output_box = aabb(
                center - glm::dvec3(radius, radius, radius),
                center + glm::dvec3(radius, radius, radius));
        return true;
    };
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

class xy_rect : public hittable {
public:
    xy_rect() {}

    xy_rect(double _x0, double _x1, double _y0, double _y1, double _k,
            shared_ptr<material> mat)
            : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k), mp(mat) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, hit_record& rec) const override{
        auto t = (k-r.origin.z) / r.direction.z;
        if (t < t_min || t > t_max)
            return false;
        auto x = r.origin.x + t*r.direction.x;
        auto y = r.origin.y + t*r.direction.y;
        if (x < x0 || x > x1 || y < y0 || y > y1)
            return false;
        rec.u = (x-x0)/(x1-x0);
        rec.v = (y-y0)/(y1-y0);
        rec.t = t;
        auto outward_normal = glm::dvec3(0, 0, 1);
        rec.set_face_normal(r, outward_normal);
        rec.mat_ptr = mp;
        rec.p = r.at(t);
        return true;
    };

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        // The bounding box must have non-zero width in each dimension, so pad the Z
        // dimension a small amount.
        output_box = aabb(glm::dvec3(x0,y0, k-0.0001), glm::dvec3(x1, y1, k+0.0001));
        return true;
    }

public:
    shared_ptr<material> mp;
    double x0, x1, y0, y1, k;
};

class xz_rect : public hittable {
public:
    xz_rect() {}

    xz_rect(double _x0, double _x1, double _z0, double _z1, double _k,
            shared_ptr<material> mat)
            : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        // The bounding box must have non-zero width in each dimension, so pad the Y
        // dimension a small amount.
        output_box = aabb(glm::dvec3(x0,k-0.0001,z0), glm::dvec3(x1, k+0.0001, z1));
        return true;
    }

public:
    shared_ptr<material> mp;
    double x0, x1, z0, z1, k;
};

class yz_rect : public hittable {
public:
    yz_rect() {}

    yz_rect(double _y0, double _y1, double _z0, double _z1, double _k,
            shared_ptr<material> mat)
            : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k), mp(mat) {};

    virtual bool hit(const Ray& r, double t_min, double t_max, hit_record& rec) const override;

    virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
        // The bounding box must have non-zero width in each dimension, so pad the X
        // dimension a small amount.
        output_box = aabb(glm::dvec3(k-0.0001, y0, z0), glm::dvec3(k+0.0001, y1, z1));
        return true;
    }

public:
    shared_ptr<material> mp;
    double y0, y1, z0, z1, k;
};

bool xz_rect::hit(const Ray& r, double t_min, double t_max, hit_record& rec) const {
    auto t = (k-r.origin.y) / r.direction.y;
    if (t < t_min || t > t_max)
        return false;
    auto x = r.origin.x + t*r.direction.x;
    auto z = r.origin.z + t*r.direction.z;
    if (x < x0 || x > x1 || z < z0 || z > z1)
        return false;
    rec.u = (x-x0)/(x1-x0);
    rec.v = (z-z0)/(z1-z0);
    rec.t = t;
    auto outward_normal = glm::dvec3(0, 1, 0);
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mp;
    rec.p = r.at(t);
    return true;
}

bool yz_rect::hit(const Ray& r, double t_min, double t_max, hit_record& rec) const {
    auto t = (k-r.origin.x) / r.direction.x;
    if (t < t_min || t > t_max)
        return false;
    auto y = r.origin.y + t*r.direction.y;
    auto z = r.origin.z + t*r.direction.z;
    if (y < y0 || y > y1 || z < z0 || z > z1)
        return false;
    rec.u = (y-y0)/(y1-y0);
    rec.v = (z-z0)/(z1-z0);
    rec.t = t;
    auto outward_normal = glm::dvec3(1, 0, 0);
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mp;
    rec.p = r.at(t);
    return true;
}

class hittable_list : public hittable {
public:
    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(shared_ptr<hittable> object) { objects.push_back(object); }

    virtual bool hit(
            const Ray& r, double t_min, double t_max, hit_record& rec) const override;
    virtual bool bounding_box(
            double time0, double time1, aabb& output_box) const override{
        if (objects.empty()) return false;

        aabb temp_box;
        bool first_box = true;

        for (const auto& object : objects) {
            if (!object->bounding_box(time0, time1, temp_box)) return false;
            output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
            first_box = false;
        }

        return true;
    };

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




glm::dvec3 ray_color(const Ray& r,const glm::dvec3 & background, const hittable& world, int depth) {
    hit_record rec;

    if (depth <= 0)
        return glm::dvec3(0,0,0);
    // If the ray hits nothing, return the background color.
    if (!world.hit(r, 0.001, infinity, rec))
        return background; //background

    Ray scattered(glm::dvec3(0),random_unit_vector());
    glm::dvec3 attenuation;

    glm::dvec3 emitted = rec.mat_ptr->emitted(rec.u, rec.v, rec.p);

    if (!rec.mat_ptr->scatter(r, rec, attenuation, scattered))
        return emitted;
    return emitted + attenuation * ray_color(scattered, background, world, depth-1);
}

int main() {

    // World
    hittable_list world;
    auto material_ground = make_shared<lambertian>(glm::dvec3(0.8, 0.2, 0.0));
    auto material_right  = make_shared<metal>(glm::dvec3(0.9725, 0.6314, 0.902));
    auto yellow_lamb = make_shared<metal>(glm::dvec3(0.8941,0.6078,0.0588));
    auto rnd_sphere = make_shared<lambertian>(glm::dvec3(0.6,0.25,0.445));
    auto difflight = make_shared<diffuse_light>(glm::dvec3(9,4,4));
    auto circle_light = make_shared<diffuse_light>(glm::dvec3(10,10,10));


    world.add(make_shared<Sphere>(glm::dvec3( 0.0, -100.5, -1.0), 100.0, material_ground));
    world.add(make_shared<Sphere>(glm::dvec3( 1.0,0.0, 1.0), 0.5, material_right));
    world.add(make_shared<Sphere>(glm::dvec3( 0.6,1.12, -1.0), 0.5, yellow_lamb));
    world.add(make_shared<Sphere>(glm::dvec3(-0.2,0.15,-0.5),0.8,rnd_sphere));
    world.add(make_shared<yz_rect>(1.5, 3, -1.5, 1.5, -2, difflight));
    world.add(make_shared<Sphere>(glm::dvec3(0,1.5,0),0.5,circle_light));

    const int max_depth = 40;

    PixelBuffer pixel_buffer(glm::ivec2(640,480));
    Camera cam(glm::dvec3(2,1,4), glm::dvec3(0,0,0),pixel_buffer);
    glm::dvec3 background(0,0,0);

    for (int y = 0; y < pixel_buffer.dimensions.y; ++y) {
        for (int x = 0; x < pixel_buffer.dimensions.x; ++x) {
            glm::dvec3 color(0);

            for (int i = 0; i < samples_per_pixel; ++i) {
                auto u = (x + random_double()) / (pixel_buffer.dimensions.x-1);
                auto v = (y + random_double()) / (pixel_buffer.dimensions.y-1);
                glm::dvec2 uv(u,v);
                Ray ray(cam.position, glm::normalize(cam.raster_to_world(uv) - cam.position));
                color+=ray_color(ray, background, world,max_depth);
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
    output.open("./task6.ppm",ios::out | ios::trunc);
    if(!output.is_open()) return 1;

    cout << "Outputting...";
    IO::write_as_PPM(pixel_buffer,output);
    output.close();
    cout << "done!" << endl;
    return 0;
}