//UTF-8
//begin
#include "rtweekend.h"

#include "bvh.h"
#include "hittable_list.h"
#include "sphere.h"
#include "cylinder.h"
#include "circle.h"
#include "triangles.h"
#include "aarect.h"
#include "camera.h"
#include "moving_sphere.h"
#include <iostream>
#include "box.h"
#include "constant_medium.h"


//目前是纯黑背景，所有光线来自于光源材质
color ray_color(
    const ray& r,
    const color& background,
    const hittable& world,
    shared_ptr<hittable> lights,
    int depth
) {
    hit_record rec;

    // If we've exceeded the ray bounce limit, no more light is gathered.
    if (depth <= 0)
        return color(0,0,0);

    // If the ray hits nothing, return the background color.
    if (!world.hit(r, 0.001, infinity, rec))
        return background;
   
    scatter_record srec;
    color emitted = rec.mat_ptr->emitted(r, rec, rec.u, rec.v, rec.p);
    
    if (!rec.mat_ptr->scatter(r, rec, srec))
        return emitted;
    if (srec.is_specular) {
        return srec.attenuation
             * ray_color(srec.specular_ray, background, world, lights, depth-1);
    }

    auto light_ptr = make_shared<hittable_pdf>(lights, rec.p);
    mixture_pdf p(light_ptr, srec.pdf_ptr);

    ray scattered = ray(rec.p, p.generate(), r.time());
    auto pdf_val = p.value(scattered.direction());


    return emitted
        + srec.attenuation * rec.mat_ptr->scattering_pdf(r, rec, scattered)
                           * ray_color(scattered, background, world, lights, depth-1) / pdf_val;

}

hittable_list cornell_box() {
    hittable_list objects;

    auto red   = make_shared<lambertian>(color(.65, .05, .05));
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto green = make_shared<lambertian>(color(.12, .45, .15));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));

    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
    objects.add(make_shared<circle>(point3(278,554,279),point3(278,0,279),60,light));
    objects.add(make_shared<circle>(point3(500,500,500),point3(0,0,0),60,light));
    //objects.add(make_shared<flip_face>(make_shared<xz_rect>(213, 343, 227, 332, 554, light)));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
    objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

    // shared_ptr<material> aluminum = make_shared<metal>(color(0.8, 0.85, 0.88), 0.0);
    // shared_ptr<hittable> box1 = make_shared<box>(point3(0,0,0), point3(165,330,165), aluminum);
    // box1 = make_shared<rotate_y>(box1, 15);
    // box1 = make_shared<translate>(box1, vec3(265,0,295));
    // objects.add(box1);

    // shared_ptr<hittable> cylinder1 = make_shared<cylinder>(point3(120,120,120), point3(120,120,0),50, green);
    // cylinder1 = make_shared<rotate_y>(cylinder1, -18);
    // cylinder1 = make_shared<translate>(cylinder1, vec3(130,0,65));
    // objects.add(cylinder1);

    shared_ptr<hittable> pyramid1=make_shared<pyramid>(point3(50,220,35),point3(90,20,30),point3(120,0,120),point3(0,0,120),red);
    pyramid1 = make_shared<rotate_y>(pyramid1, 45);
    pyramid1 = make_shared<translate>(pyramid1, vec3(350,100,350));
    objects.add(pyramid1);
    // shared_ptr<hittable> circle1 = make_shared<circle>(point3(120,120,120), point3(120,0,120),50, green);
    // objects.add(circle1);

    // shared_ptr<hittable> triangle1=make_shared<triangle>(vec3(113, 54, 127), vec3(243, 54, 127), vec3(178, 54, 232),green);
    // objects.add(triangle1);
    // auto glass = make_shared<dielectric>(1.5);
    // objects.add(make_shared<sphere>(point3(190,90,190), 90 , glass));
    //return objects;
    return static_cast<hittable_list>(make_shared<bvh_node>(objects,0,1));
}


//main.cc
int main() {

    // Image
    const auto aspect_ratio = 1.0 / 1.0;
    const int image_width = 600;
    const int image_height = static_cast<int>(image_width / aspect_ratio);
    const int samples_per_pixel = 10;
    const int max_depth = 50;


    // World
    //shared_ptr<hittable> lights =
    //make_shared<xz_rect>(213, 343, 227, 332, 554, shared_ptr<material>());
    //make_shared<sphere>(point3(190, 90, 190), 90, shared_ptr<material>());
    auto lights = make_shared<hittable_list>();
    lights->add(make_shared<circle>(point3(278,554,279),point3(278,0,279),60,shared_ptr<material>()));
    lights->add(make_shared<circle>(point3(500,500,500),point3(0,0,0),60,shared_ptr<material>()));
    //lights->add(make_shared<xz_rect>(213, 343, 227, 332, 554, shared_ptr<material>()));
    //lights->add(make_shared<sphere>(point3(190, 90, 190), 90, shared_ptr<material>()));
    auto world = cornell_box();
    color background(0,0,0);

    // Camera

    point3 lookfrom(278, 278, -800);
    point3 lookat(278, 278, 0);
    vec3 vup(0, 1, 0);
    auto dist_to_focus = 10.0;
    auto aperture = 0.0;
    auto vfov = 40.0;
    auto time0 = 0.0;
    auto time1 = 1.0;
    
    camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, time0, time1);

    //camera cam(point3(-2,2,1), point3(0,0,-1), vec3(0,1,0), 90, aspect_ratio);
  
    // Render

    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";
    for (int j = image_height-1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            vec3 color(0, 0, 0);
            for (int s = 0; s < samples_per_pixel; ++s) {
                auto u = (i + random_double()) / image_width;
                auto v = (j + random_double()) / image_height;
                ray r = cam.get_ray(u, v);
                color += ray_color(r, background, world, lights,max_depth);
            }
            color.write_color(std::cout, samples_per_pixel);
        }
    }

    std::cerr << "\nDone.\n";
}