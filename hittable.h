//hittable.h
#ifndef HITTABLE_H
#define HITTABLE_H

#include "rtweekend.h"
#include "aabb.h"
class material;
struct hit_record {
    vec3 p;//命中的平面的交点
    vec3 normal;//命中的交点的法向量，法相与入射方向相反
    shared_ptr<material> mat_ptr;
    double t;//记录光线光线命中的参数，用来保证最近的物体才能反射（远处的会被遮挡）
    double u;//表示纹理坐标，xy中的x
    double v;//表示纹理坐标，xy中的y
    bool front_face; // true if ray is from inside the object 
    inline void set_face_normal(const ray&r,const vec3& outward_normal) {//outward_normal表示向外的法相；
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable {
    public:
        virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const = 0;//t表示交点，ray(t)
        virtual bool bounding_box(double time0, double time1, aabb& output_box)const = 0;
};

#endif