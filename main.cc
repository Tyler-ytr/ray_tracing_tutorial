#include "rtweekend.h"


#include "hittable_list.h"
#include "sphere.h"
#include "camera.h"

#include <iostream>


vec3 ray_color(const ray& r, const hittable& world,int depth) {//返回的值是rgb颜色
    hit_record rec;

    if(depth<=0){
        return vec3(0,0,0);//返回黑色
    }
    if (world.hit(r, 0.001, infinity, rec)) {//本来是0，防止自相交
        //vec3 target = rec.p + random_in_hemisphere(rec.normal);//从入射点开始选取一个随机的方向, 然后再判断是否在法向量所在的那个半球。
        vec3 target = rec.p + rec.normal + random_unit_vector();//单位球的球面取点然后单位化
        //vec3 target = rec.p + rec.normal + random_in_unit_sphere();//单位球的球心加上一个随机值
        return 0.5*ray_color(ray(rec.p, target-rec.p), world, depth-1);
        
        // return 0.5 * (rec.normal + vec3(1,1,1));
    }
    vec3 unit_direction = unit_vector(r.direction());
    auto t = 0.5*(unit_direction.y() + 1.0);
    return (1.0-t)*vec3(1.0, 1.0, 1.0) + t*vec3(0.5, 0.7, 1.0);
}

int main() {
    const int image_width = 200;
    const int image_height = 100;
    const int samples_per_pixel = 100;
    const int max_depth = 50;//漫反射最大深度

    std::cout << "P3\n" << image_width << " " << image_height << "\n255\n";
       
    hittable_list world;
    world.add(make_shared<sphere>(vec3(0,0,-1), 0.5));
    world.add(make_shared<sphere>(vec3(0,-100.5,-1), 100)); //一个特别大的球，作为地板
    camera cam;
    
    for (int j = image_height-1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            vec3 color(0, 0, 0);
            for (int s = 0; s < samples_per_pixel; ++s) {//多次采样
                auto u = (i + random_double()) / (image_width-1);
                auto v = (j + random_double()) / (image_height-1);
                ray r = cam.get_ray(u, v);
                color += ray_color(r, world, max_depth);
            }
            color.write_color(std::cout, samples_per_pixel);
        }
    }

    std::cerr << "\nDone.\n";
}