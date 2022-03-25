//sphere.h
#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "vec3.h"
class sphere: public hittable{
    public:
        sphere(){};
        sphere(vec3 cen,double r,shared_ptr<material> m):center(cen),radius(r),mat_ptr(m) {};;
        virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const;

    public:
        vec3 center;
        double radius;
        shared_ptr<material> mat_ptr;
};
bool sphere::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    vec3 oc = r.origin() - center;
    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());
    auto c = oc.length_squared() - radius*radius;
    auto discriminant = half_b*half_b - a*c;

    if(discriminant>0){
        auto root=sqrt(discriminant);
        auto temp = (-half_b - root)/a;//小的根 ok就用小的 直线上面的点p(temp)与球体相交
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;//求向外的法向量
            rec.set_face_normal(r, outward_normal);//设置法向量
            rec.mat_ptr = mat_ptr;
            return true;
        }

        temp = (-half_b + root) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;//求向外的法向量
            rec.set_face_normal(r, outward_normal);//设置法向量
            rec.mat_ptr = mat_ptr;
            return true;
        }


    }
    return false;


}


#endif