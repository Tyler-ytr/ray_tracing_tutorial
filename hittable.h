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
//通过移动射线的方式来完成位置的变换，也就是说先把光线调整到变换之前的位置，然后计算hit record，最后再整体加上去（先减后加）
class translate : public hittable {//平移，输出的是加上offset之后的物体
    public:
        translate(shared_ptr<hittable> p, const vec3& displacement)
            : ptr(p), offset(displacement) {}

        virtual bool hit(
            const ray& r, double t_min, double t_max, hit_record& rec) const override;

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

    public:
        shared_ptr<hittable> ptr;
        vec3 offset;
};

bool translate::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    ray moved_r(r.origin() - offset, r.direction(), r.time());//把光线移动到变换之前的位置
    if (!ptr->hit(moved_r, t_min, t_max, rec))//调用之前对象的hit函数计算
        return false;

    rec.p += offset;//计算好的结果加上来
    rec.set_face_normal(moved_r, rec.normal);//法相不变

    return true;
}

bool translate::bounding_box(double time0, double time1, aabb& output_box) const {
    if (!ptr->bounding_box(time0, time1, output_box))
        return false;

    output_box = aabb(
        output_box.min() + offset,
        output_box.max() + offset);//包围盒直接加上offset就行了

    return true;
}

class rotate_y : public hittable {
    //绕y轴旋转
    public:
        rotate_y(shared_ptr<hittable> p, double angle);//输入实体以及旋转的角度(degree)

        virtual bool hit(
            const ray& r, double t_min, double t_max, hit_record& rec) const override;

        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override {
            output_box = bbox;
            return hasbox;
        }

    public:
        shared_ptr<hittable> ptr;
        double sin_theta;
        double cos_theta;
        bool hasbox;
        aabb bbox;//旋转之后的包围盒
};
rotate_y::rotate_y(shared_ptr<hittable> p, double angle) : ptr(p) {
    auto radians = degrees_to_radians(angle);
    sin_theta = sin(radians);
    cos_theta = cos(radians);
    hasbox = ptr->bounding_box(0, 1, bbox);

    point3 min( infinity,  infinity,  infinity);
    point3 max(-infinity, -infinity, -infinity);
    //枚举包围盒 最大值 (加 减 不加不减) 最小值 三个维度
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                auto x = i*bbox.max().x() + (1-i)*bbox.min().x();
                auto y = j*bbox.max().y() + (1-j)*bbox.min().y();
                auto z = k*bbox.max().z() + (1-k)*bbox.min().z();

                auto newx =  cos_theta*x + sin_theta*z;
                auto newz = -sin_theta*x + cos_theta*z;

                vec3 tester(newx, y, newz);

                for (int c = 0; c < 3; c++) {
                    min[c] = fmin(min[c], tester[c]);
                    max[c] = fmax(max[c], tester[c]);
                }
            }
        }
    }

    bbox = aabb(min, max);
}
bool rotate_y::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    auto origin = r.origin();
    auto direction = r.direction();
    //相当于公式里面的theta取负数，原因和平移类似
    origin[0] = cos_theta*r.origin()[0] - sin_theta*r.origin()[2];
    origin[2] = sin_theta*r.origin()[0] + cos_theta*r.origin()[2];

    direction[0] = cos_theta*r.direction()[0] - sin_theta*r.direction()[2];
    direction[2] = sin_theta*r.direction()[0] + cos_theta*r.direction()[2];

    ray rotated_r(origin, direction, r.time());

    if (!ptr->hit(rotated_r, t_min, t_max, rec))
        return false;

    auto p = rec.p;
    auto normal = rec.normal;

    p[0] =  cos_theta*rec.p[0] + sin_theta*rec.p[2];
    p[2] = -sin_theta*rec.p[0] + cos_theta*rec.p[2];

    normal[0] =  cos_theta*rec.normal[0] + sin_theta*rec.normal[2];
    normal[2] = -sin_theta*rec.normal[0] + cos_theta*rec.normal[2];

    rec.p = p;
    rec.set_face_normal(rotated_r, normal);

    return true;
}

#endif