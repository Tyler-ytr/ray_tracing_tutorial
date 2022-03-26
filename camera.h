#ifndef CAMERA_H
#define CAMERA_H

#include "rtweekend.h"

class camera {
    public:
        camera(
            point3 lookfrom, //从这点看
            point3 lookat, //看向这点
            vec3   vup, //垂直向量
            double vfov, // vertical field-of-view in degrees,视野范围(field of view, fov)
            double aspect_ratio,// 我认为是宽高比
            double aperture,//光圈
            double focus_dist //焦点距离
        ) {
            auto theta = degrees_to_radians(vfov);//角度转弧度
            auto h = tan(theta/2);
            auto viewport_height = 2.0 * h;
            auto viewport_width = aspect_ratio * viewport_height;

            w = unit_vector(lookfrom - lookat);//摄像机看向-w
            u = unit_vector(cross(vup, w));
            v = cross(w, u);            

            

            origin = lookfrom;
            horizontal = focus_dist * viewport_width * u;
            vertical = focus_dist * viewport_height * v;
            lower_left_corner = origin - horizontal/2 - vertical/2 - focus_dist*w;
        
            lens_radius = aperture / 2;
        }

        ray get_ray(double s, double t) const {
            vec3 rd = lens_radius * random_in_unit_disk();
            vec3 offset = u * rd.x() + v * rd.y();

            return ray(
                origin + offset,
                lower_left_corner + s*horizontal + t*vertical - origin - offset
            );
        }

    private:
        point3 origin;
        point3 lower_left_corner;
        vec3 horizontal;
        vec3 vertical;
        vec3 u, v, w;
        double lens_radius;
};



#endif