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
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;
        virtual double pdf_value(const point3& o, const vec3& v) const;
        virtual vec3 random(const point3& o) const;
    public:
        vec3 center;
        double radius;
        shared_ptr<material> mat_ptr;
    private:
        static void get_sphere_uv(const point3& p, double& u, double& v) {
            // p: a given point on the sphere of radius one, centered at the origin.
            // u: returned value [0,1] of angle around the Y axis from X=-1.
            // v: returned value [0,1] of angle from Y=-1 to Y=+1.
            //     <1 0 0> yields <0.50 0.50>       <-1  0  0> yields <0.00 0.50>
            //     <0 1 0> yields <0.50 1.00>       < 0 -1  0> yields <0.50 0.00>
            //     <0 0 1> yields <0.25 0.50>       < 0  0 -1> yields <0.75 0.50>

            auto theta = acos(-p.y());
            auto phi = atan2(-p.z(), p.x()) + pi;

            u = phi / (2*pi);
            v = theta / pi;
        }
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
            get_sphere_uv(outward_normal, rec.u, rec.v);
            rec.mat_ptr = mat_ptr;
            return true;
        }

        temp = (-half_b + root) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;//求向外的法向量
            rec.set_face_normal(r, outward_normal);//设置法向量
            get_sphere_uv(outward_normal, rec.u, rec.v);
            rec.mat_ptr = mat_ptr;
            return true;
        }
    }
    return false;
}

bool sphere::bounding_box(double time0, double time1, aabb& output_box) const {
    output_box = aabb(
        center - vec3(radius, radius, radius),
        center + vec3(radius, radius, radius));
    return true;
}

double sphere::pdf_value(const point3& o, const vec3& v) const {
    hit_record rec;
    if (!this->hit(ray(o, v), 0.001, infinity, rec))
        return 0;

    auto cos_theta_max = sqrt(1 - radius*radius/(center-o).length_squared());
    auto solid_angle = 2*pi*(1-cos_theta_max);

    return  1 / solid_angle;
}

vec3 sphere::random(const point3& o) const {
    vec3 direction = center - o;
    auto distance_squared = direction.length_squared();
    onb uvw;
    uvw.build_from_w(direction);
    return uvw.local(random_to_sphere(radius, distance_squared));
}
#endif