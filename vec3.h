#ifndef VEC3_H
#define VEC3_H

#include <cmath>
#include <iostream>
#include "rtweekend.h"

using std::sqrt;

class vec3 {
    public:
        vec3() : e{0,0,0} {}
        vec3(double e0, double e1, double e2) : e{e0, e1, e2} {}

        double x() const { return e[0]; }
        double y() const { return e[1]; }
        double z() const { return e[2]; }

        vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
        double operator[](int i) const { return e[i]; }
        double& operator[](int i) { return e[i]; }

        vec3& operator+=(const vec3 &v) {
            e[0] += v.e[0];
            e[1] += v.e[1];
            e[2] += v.e[2];
            return *this;
        }

        vec3& operator*=(const double t) {
            e[0] *= t;
            e[1] *= t;
            e[2] *= t;
            return *this;
        }

        vec3& operator/=(const double t) {
            return *this *= 1/t;
        }

        double length() const {
            return sqrt(length_squared());
        }

        double length_squared() const {
            return e[0]*e[0] + e[1]*e[1] + e[2]*e[2];
        }
        void write_color(std::ostream &out, int samples_per_pixel) {
            // Divide the color total by the number of samples. 多次采样之后求平均值
            auto scale = 1.0 / samples_per_pixel;
            auto r =  e[0];
            auto g = e[1];
            auto b =  e[2];
            // Replace NaN components with zero. See explanation in Ray Tracing: The Rest of Your Life.
            if (r != r) r = 0.0;
            if (g != g) g = 0.0;
            if (b != b) b = 0.0;
            
            //伽马矫正，之前没有sqrt
            r = sqrt(scale * r);
            g = sqrt(scale * g);
            b = sqrt(scale * b);

            // Write the translated [0,255] value of each color component.
            out << static_cast<int>(256 * clamp(r, 0.0, 0.999)) << ' '
                << static_cast<int>(256 * clamp(g, 0.0, 0.999)) << ' '
                << static_cast<int>(256 * clamp(b, 0.0, 0.999)) << '\n';
        }
        inline static vec3 random(){
            return vec3(random_double(), random_double(), random_double());
        }
        inline static vec3 random(double min, double max) {
            return vec3(random_double(min,max), random_double(min,max), random_double(min,max));
        }
        bool near_zero() const {
        // 如果向量在各个维度都接近零返回true.
        const auto s = 1e-8;
        return (fabs(e[0]) < s) && (fabs(e[1]) < s) && (fabs(e[2]) < s);
        }


    public:
        double e[3];
};

// Type aliases for vec3
using point3 = vec3;   // 3D point
using color = vec3;    // RGB color

inline std::ostream& operator<<(std::ostream &out, const vec3 &v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline vec3 operator+(const vec3 &u, const vec3 &v) {
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline vec3 operator-(const vec3 &u, const vec3 &v) {
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline vec3 operator*(const vec3 &u, const vec3 &v) {
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline vec3 operator*(double t, const vec3 &v) {
    return vec3(t*v.e[0], t*v.e[1], t*v.e[2]);
}

inline vec3 operator*(const vec3 &v, double t) {
    return t * v;
}

inline vec3 operator/(vec3 v, double t) {
    return (1/t) * v;
}

inline double dot(const vec3 &u, const vec3 &v) {
    return u.e[0] * v.e[0]
         + u.e[1] * v.e[1]
         + u.e[2] * v.e[2];
}

inline vec3 cross(const vec3 &u, const vec3 &v) {
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline vec3 unit_vector(vec3 v) {
    return v / v.length();
}


vec3 random_in_unit_sphere() {
    // 在一个xyz取值范围为-1到+1的单位立方体中选取一个随机点, 如果这个点在球外就重新生成直到该点在球内:
    while (true) {
        auto p = vec3::random(-1,1);
        if (p.length_squared() >= 1) continue;
        return p;
    }
}

vec3 random_unit_vector(){
    //极坐标生成随机向量,上面的方法是单位球体里面取得，这个是在球面上取得然后长度单位化
    auto a=random_double(0,2*pi);
    auto z=random_double(-1,1);
    auto r=sqrt(1-z*z);
    return vec3(r*cos(a), r*sin(a), z);
}

vec3 random_in_hemisphere(const vec3& normal) {
            //在入射点开始选取一个随机的方向, 然后再判断是否在法向量所在的那个半球。
            vec3 in_unit_sphere = random_in_unit_sphere();
            if (dot(in_unit_sphere, normal) > 0.0) // In the same hemisphere as the normal
                return in_unit_sphere;
            else
                return -in_unit_sphere;
        }

vec3 reflect(const vec3& v, const vec3& n) {
    return v - 2*dot(v,n)*n;//反射，n是平面的法向量，v是入射的光线，返回反射的光线
}
//折射
vec3 refract(const vec3& uv,const vec3&n,double etai_over_etat){//uv是单位化的入射光线，n是平面的法向量，etai_over_etat是折射率的比值(上面的介质比下面的介质)
    auto cos_theta=dot(-uv,n);
    vec3 r_out_parallel =  etai_over_etat * (uv + cos_theta*n);//平行的向量
    vec3 r_out_perp = -sqrt(1.0 - r_out_parallel.length_squared()) * n;//垂直的向量
    return r_out_parallel + r_out_perp;
}

vec3 random_in_unit_disk() {
    while (true) {
        auto p = vec3(random_double(-1,1), random_double(-1,1), 0);
        if (p.length_squared() >= 1) continue;
        return p;
    }
}//圆盘的随机点

// inline vec3 random_cosine_direction() {
//     auto r1 = random_double();
//     auto r2 = random_double();
//     auto z = sqrt(1-r2);

//     auto phi = 2*pi*r1;
//     auto x = cos(phi)*sqrt(r2);
//     auto y = sin(phi)*sqrt(r2);

//     return vec3(x, y, z);
// }
#endif