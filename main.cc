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
#include "sample.h"


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
    auto pertext = make_shared<lambertian>(make_shared<noise_texture>(4));
    auto earth_texture = make_shared<image_texture>("picture/earthmap.jpg");
    auto earth_surface = make_shared<lambertian>(earth_texture);
    auto blue=make_shared<lambertian>(color(0.2, 0.4, 0.9));


    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
    objects.add(make_shared<circle>(point3(278,554,279),point3(278,0,279),60,light));
    objects.add(make_shared<circle>(point3(500,500,500),point3(0,0,0),60,light));
    objects.add(make_shared<circle>(point3(60,500,500),point3(500,0,0),60,light));
    //objects.add(make_shared<flip_face>(make_shared<xz_rect>(213, 343, 227, 332, 554, light)));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 555, white));
    objects.add(make_shared<xz_rect>(0, 555, 0, 555, 0, white));
    objects.add(make_shared<xy_rect>(0, 555, 0, 555, 555, white));

    shared_ptr<material> aluminum = make_shared<metal>(color(0.8, 0.85, 0.88), 0.0);
    shared_ptr<hittable> box1 = make_shared<box>(point3(0,0,0), point3(120,330,120), aluminum);
    box1 = make_shared<rotate_y>(box1, 10);
    box1 = make_shared<translate>(box1, vec3(400,0,295));
    objects.add(box1);

    shared_ptr<hittable> cylinder1 = make_shared<cylinder>(point3(0,0,0), point3(120,120,120),50,white);
    cylinder1 = make_shared<rotate_y>(cylinder1, -18);
    cylinder1 = make_shared<translate>(cylinder1, vec3(160,230,80));
    objects.add(cylinder1);

    shared_ptr<hittable> pyramid1=make_shared<pyramid>(point3(0,140,0),point3(-1.44*70,0,-1.44*70),point3(140,0,0),point3(0,0,140),blue);
    pyramid1 = make_shared<rotate_y>(pyramid1, 70);
    pyramid1 = make_shared<translate>(pyramid1, vec3(150,0,350));
    objects.add(pyramid1);

    auto glass = make_shared<dielectric>(1.5);
    objects.add(make_shared<sphere>(point3(350,90,190), 50 , glass));

    objects.add(make_shared<sphere>(point3(410,50,120), 50 , earth_surface));
    auto moving_sphere_material = make_shared<lambertian>(color(0.7, 0.3, 0.1));
    vec3 center1 = point3(290, 140, 260);
    objects.add(make_shared<moving_sphere>(center1, center1+vec3(30,0,0), 0, 1, 50, moving_sphere_material));


    return static_cast<hittable_list>(make_shared<bvh_node>(objects,0,1));
}
hittable_list chess_board() {
    hittable_list world;
    auto white = make_shared<lambertian>(color(.73, .73, .73));
    auto light = make_shared<diffuse_light>(color(15, 15, 15));
    world.add(make_shared<circle>(point3(100,100,100),point3(0,0,0),50,light));
    auto checker = make_shared<checker_texture>(color(0.2, 0.3, 0.1), color(0.9, 0.9, 0.9));
    world.add(make_shared<sphere>(point3(0,-1000,0), 1000, make_shared<lambertian>(checker)));

    int i = 1;


    // world.add(make_shared<sphere>(vec3(0, 1, 0), 1.0, make_shared<dielectric>(1.5)));

    // world.add(
    //     make_shared<sphere>(vec3(-4, 1, 0), 1.0, make_shared<lambertian>(vec3(0.4, 0.2, 0.1))));

    // world.add(
    //     make_shared<sphere>(vec3(4, 1, 0), 1.0, make_shared<metal>(vec3(0.7, 0.6, 0.5), 0.0)));

    return static_cast<hittable_list>(make_shared<bvh_node>(world,0,1));
}


//main.cc
int main() {

    // Image
    const auto aspect_ratio = 1.0 / 1.0;
    const int image_width = 600;
    const int image_height = static_cast<int>(image_width / aspect_ratio);
    //const int samples_per_pixel = 10;
    const int sample_threshold=25;//100*100的画布里面间隔为25做采样，然后归一化，相当于4*4次采样
    const int sample_type=2;//0:uniform,1:random,2:Fastpoisson(blue noise)
    const int max_depth = 50;


    // World

    auto lights = make_shared<hittable_list>();
    //chess board's light
    lights->add(make_shared<circle>(point3(100,100,100),point3(0,0,0),50,shared_ptr<material>()));
    
    // lights->add(make_shared<circle>(point3(500,500,500),point3(0,0,0),60,shared_ptr<material>()));
    // lights->add(make_shared<circle>(point3(60,500,500),point3(500,0,0),60,shared_ptr<material>()));
    //lights->add(make_shared<sphere>(point3(350,50,190), 90, shared_ptr<material>()));
    //auto world = cornell_box();
    auto world =chess_board();
    color background(0,0,0);

    // Camera

    // point3 lookfrom(278, 278, -800);
    // point3 lookat(278, 278, 0);
    // vec3 vup(0, 1, 0);
    vec3 lookfrom(13,2,3);
    vec3 lookat(0,0,0);
    vec3 vup(0,1,0);
    auto dist_to_focus = 10.0;
    auto aperture = 0.0;
    auto vfov = 40.0;
    auto time0 = 0.0;
    auto time1 = 1.0;
    
    camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, time0, time1);

  
    // Render
    // initial sampler

    Sampler sampler(sample_threshold,sample_type);
    std::vector<std::pair<double,double>> randomlist;
    int samples_per_pixel=10;//10为默认参数没有意义
    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";
    for (int j = image_height-1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            vec3 color(0, 0, 0);
            randomlist=sampler.sampling();
            samples_per_pixel=randomlist.size();
            for (int s = 0; s < samples_per_pixel; ++s) {
                auto u = (i + randomlist[s].first) / image_width;
                auto v = (j + randomlist[s].second) / image_height;
                ray r = cam.get_ray(u, v);
                color += ray_color(r, background, world, lights,max_depth);
            }
            color.write_color(std::cout, samples_per_pixel);
        }
    }

    std::cerr << "\nDone.\n";
}