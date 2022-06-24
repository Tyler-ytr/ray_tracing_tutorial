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

    // auto red   = make_shared<lambertian>(color(.65, .05, .05));
    // auto white = make_shared<lambertian>(color(.73, .73, .73));
    // auto green = make_shared<lambertian>(color(.12, .45, .15));
    // auto light = make_shared<diffuse_light>(color(15, 15, 15));
    // auto pertext = make_shared<lambertian>(make_shared<noise_texture>(4));
    // auto earth_texture = make_shared<image_texture>("picture/earthmap.jpg");
    // auto earth_surface = make_shared<lambertian>(earth_texture);
    // auto blue=make_shared<lambertian>(color(0.2, 0.4, 0.9));

    auto red   = make_shared<Microfast>(color(.65, .05, .05),0.6,1.2);
    auto white = make_shared<Microfast>(color(.73, .73, .73),0.6,1.2);
    auto green = make_shared<Microfast>(color(.12, .45, .15),0.6,1.2);
    auto light = make_shared<diffuse_light>(color(15, 15, 15));
    auto pertext = make_shared<lambertian>(make_shared<noise_texture>(4));
    auto earth_texture = make_shared<image_texture>("picture/earthmap.jpg");
    auto earth_surface = make_shared<lambertian>(earth_texture);
    auto blue=make_shared<Microfast>(color(0.2, 0.4, 0.9),0.6,1.2);


    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 555, green));
    objects.add(make_shared<yz_rect>(0, 555, 0, 555, 0, red));
    objects.add(make_shared<polygon>(light,6,vec3(400, 554, 250), vec3(300, 554, 200), vec3(200, 554, 250),vec3(200, 554, 350),vec3(300, 554, 400),vec3(400, 554, 350)));
    //objects.add(make_shared<triangle>(vec3(213, 554, 227), vec3(343, 554, 227), vec3(278, 554, 332), light));
    //objects.add(make_shared<circle>(point3(278,554,279),point3(278,0,279),60,light));
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
    auto white_9 = make_shared<Microfast>(color(0.4, 0.275, 0.05),0.9,1.2);
    shared_ptr<hittable> cylinder1 = make_shared<cylinder>(point3(0,0,0), point3(30,270,30),20,white_9);
    cylinder1 = make_shared<rotate_y>(cylinder1, -18);
    cylinder1 = make_shared<translate>(cylinder1, vec3(50,0,120));
    objects.add(cylinder1);

    shared_ptr<hittable> pyramid1=make_shared<pyramid>(point3(0,140,0),point3(-1.44*70,0,-1.44*70),point3(140,0,0),point3(0,0,140),blue);
    pyramid1 = make_shared<rotate_y>(pyramid1, 70);
    pyramid1 = make_shared<translate>(pyramid1, vec3(150,0,350));
    objects.add(pyramid1);

    auto Microfast_1_12  = make_shared<Microfast>(color(0.2, 0.4, 0.9),0.1,1.2);//蓝色
    objects.add(make_shared<sphere>(point3(410,50,120), 50 , Microfast_1_12 ));
    auto Microfast_3_12  = make_shared<Microfast>(color(0.3, 0.375, 0.7),0.3,1.2);
    objects.add(make_shared<sphere>(point3(360,100,170), 50 , Microfast_3_12));
    auto Microfast_5_12  = make_shared<Microfast>(color(0.4, 0.35, 0.5),0.5,1.2);
    objects.add(make_shared<sphere>(point3(310, 150, 220), 50 , Microfast_5_12 ));
    auto Microfast_7_12  = make_shared<Microfast>(color(0.5, 0.325, 0.3),0.7,1.2);
    objects.add(make_shared<sphere>(point3(260, 200, 270), 50 , Microfast_7_12 ));
    auto Microfast_9_12  = make_shared<Microfast>(color(0.6, 0.3, 0.1),0.9,1.2);
    objects.add(make_shared<sphere>(point3(210, 250, 320), 50 , Microfast_9_12 )); 
    
    
    return static_cast<hittable_list>(make_shared<bvh_node>(objects,0,1));
}
hittable_list BRDFtest(){
    hittable_list objects;
    hittable_list boxes1;
    

    auto light = make_shared<diffuse_light>(color(15, 15, 15));
    auto ground = make_shared<lambertian>(color(0.48, 0.83, 0.53));


    const int boxes_per_side = 20;
    for (int i = 0; i < boxes_per_side; i++) 
    {
        for (int j = 0; j < boxes_per_side; j++) 
        {
            auto w = 100.0;
            auto x0 = -1000.0 + i*w;
            auto z0 = -1000.0 + j*w;
            auto y0 = 0.0;
            auto x1 = x0 + w;
            auto y1 = random_double(1,101);
            auto z1 = z0 + w;

            boxes1.add(make_shared<box>(point3(x0,y0,z0), point3(x1,y1,z1), ground));
        }
    }
    objects.add(make_shared<bvh_node>(boxes1, 0, 1));
    //光源
    // objects.add(make_shared<xz_rect>(123, 423, 147, 412, 554, light));
    objects.add(make_shared<circle>(point3(200,500,200),point3(200,0,200),150,light));
    // objects.add(make_shared<polygon>(light,4,vec3(250, 554,400), 
    // vec3(350, 554, 400), vec3(400, 554,300),vec3(350, 554, 200),vec3(250, 554, 200),vec3(200, 554, 300)));
    
    auto center1 = point3(400, 400, 200);
    auto center2 = center1 + vec3(30,0,0);
    //左上角的黄色的球
    auto Microfast_6_12  = make_shared<Microfast>(color(0.7, 0.3, 0.1),0.6,1.2);
    objects.add(make_shared<sphere>(point3(400, 200, 200), 50, Microfast_6_12));
    //
    auto Microfast_2_12  = make_shared<Microfast>(color(0.2, 0.4, 0.9),0.1,1.2);
    objects.add(make_shared<sphere>(point3(260, 150, 45), 50, Microfast_2_12));



    return static_cast<hittable_list>(make_shared<bvh_node>(objects,0,1));
}

//main.cc
int main() {

    // Image
    const auto aspect_ratio = 1.0 / 1.0;
    const int image_width = 600;
    const int image_height = static_cast<int>(image_width / aspect_ratio);
    const int samples_per_pixel = 1000;
    const int max_depth = 50;


    // World
    // cornell_box World
    auto lights = make_shared<hittable_list>();

    lights->add(make_shared<polygon>(shared_ptr<material>(),6,vec3(400, 554, 250), vec3(300, 554, 200), vec3(200, 554, 250),vec3(200, 554, 350),vec3(300, 554, 400),vec3(400, 554, 350)));
    //lights->add(make_shared<triangle>(vec3(213, 554, 227), vec3(343, 554, 227), vec3(278, 554, 332),shared_ptr<material>()));
    //lights->add(make_shared<circle>(point3(278,554,279),point3(278,0,279),60,shared_ptr<material>()));
    lights->add(make_shared<circle>(point3(500,500,500),point3(0,0,0),60,shared_ptr<material>()));
    lights->add(make_shared<circle>(point3(60,500,500),point3(500,0,0),60,shared_ptr<material>()));
    //lights->add(make_shared<sphere>(point3(350,50,190), 90, shared_ptr<material>()));
    auto world = cornell_box();


    // BRDF testing World
    // lights->add(make_shared<circle>(point3(200,500,200),point3(200,0,200),150,shared_ptr<material>()));
    // auto world = BRDFtest();

    color background(0,0,0);

    // cornell box Camera

    point3 lookfrom(278, 278, -800);
    point3 lookat(278, 278, 0);
    vec3 vup(0, 1, 0);
    auto dist_to_focus = 10.0;
    auto aperture = 0.0;
    auto vfov = 40.0;
    auto time0 = 0.0;
    auto time1 = 1.0;

    // BRDF testing Camera
    // point3 lookfrom(478, 278, -600);
    // point3 lookat(278, 278, 0);
    // vec3 vup(0, 1, 0);
    // auto dist_to_focus = 10.0;
    // auto aperture = 0.0;
    // auto vfov = 40.0;
    // auto time0 = 0.0;
    // auto time1 = 1.0;
    
    camera cam(lookfrom, lookat, vup, vfov, aspect_ratio, aperture, dist_to_focus, time0, time1);

  
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