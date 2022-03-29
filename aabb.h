#ifndef AABB_H
#define AABB_H

#include "rtweekend.h"

class aabb {
    public:
        aabb() {}
        aabb(const point3& a, const point3& b) { minimum = a; maximum = b;}//用这两个点代表了一个包围盒，一个点相当于三个面

        point3 min() const {return minimum; }
        point3 max() const {return maximum; }

        bool hit(const ray& r, double t_min, double t_max)const;
        aabb surrounding_box(aabb box0, aabb box1);

        point3 minimum;
        point3 maximum;
};
inline bool aabb::hit(const ray& r, double t_min, double t_max) const {//t是射线和包围盒面的交点。
    for (int a = 0; a < 3; a++) {
        auto invD = 1.0f / r.direction()[a];
        auto t0 = (min()[a] - r.origin()[a]) * invD;
        auto t1 = (max()[a] - r.origin()[a]) * invD;
        if (invD < 0.0f)
            std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max <= t_min)
            return false;
    }
    return true;

    //和下面的等价，但是快一点：
        // bool hit(const ray& r, double t_min, double t_max) const {
        //     for (int a = 0; a < 3; a++) {
        //         auto t0 = fmin((minimum[a] - r.origin()[a]) / r.direction()[a],
        //                        (maximum[a] - r.origin()[a]) / r.direction()[a]);
        //         auto t1 = fmax((minimum[a] - r.origin()[a]) / r.direction()[a],
        //                        (maximum[a] - r.origin()[a]) / r.direction()[a]);
        //         t_min = fmax(t0, t_min);
        //         t_max = fmin(t1, t_max);
        //         if (t_max <= t_min)
        //             return false;
        //     }
        //     return true;
        // }   
}
aabb surrounding_box(aabb box0, aabb box1) {//求两个包围盒的包围盒
    point3 small(fmin(box0.min().x(), box1.min().x()),
                 fmin(box0.min().y(), box1.min().y()),
                 fmin(box0.min().z(), box1.min().z()));

    point3 big(fmax(box0.max().x(), box1.max().x()),
               fmax(box0.max().y(), box1.max().y()),
               fmax(box0.max().z(), box1.max().z()));

    return aabb(small,big);
}
#endif