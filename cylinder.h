/*******************************************************************
 * @Descripttion   : 圆柱体
 * @version        : 
 * @Author         : tyler-ytr
 * @Date           : 2022-04-01 21:15
 * @LastEditTime   : 2022-04-01 21:39
*******************************************************************/
//cylinder.h
#ifndef CYLINDER_H
#define CYLINDER_H
#include "hittable.h"
#include "vec3.h"

class cylinder: public hittable{
    public:
        enum Dir
        {
            xAxis,
            yAxis,
            zAxis
        };//表示方向
        cylinder():center({ 0,0,0 }), radius(1.0), height(1.0), d(yAxis), matPtr(nullptr) { halfHeight = 0.0; }
        cylinder(vec3 c, double r, double h, Dir _d, shared_ptr<material> p):center(c),radius(r),height(h),d(_d),mat_ptr(p){halfHeight = height / 2.0;};
        virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;
        // virtual double pdf_value(const point3& o, const vec3& v) const;
        // virtual vec3 random(const point3& o) const;
    private:
        vec3 center;
        double halfHeight;
        double height;
        double radius;
        shared_ptr<material> mat_ptr;
        Dir d;

};

bool  cylinder::hit(const ray& r, double tmin, double tmax, hit_record& rec) const{
    ;
}

#endif