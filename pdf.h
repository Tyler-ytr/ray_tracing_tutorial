/*
 * @Author: Tyler-ytr
 * @Date: 2022-04-01 17:56:58
 * @LastEditors: Tyler-ytr
 * @LastEditTime: 2022-04-01 18:51:39
 * @FilePath: /raytracing/ray_tracing_tutorial/pdf.h
 * @Description: probability density function;In the Monte Carlo system, pdf acts as the most important element of Important-Sample
 * 
 * Copyright (c) 2022 by Tyler-ytr, All Rights Reserved. 
 */
#ifndef PDF_H
#define PDF_H
#include "rtweekend.h"
#include "vec3.h"

class pdf {
    public:
        virtual ~pdf() {}
        /*******************************************************************
         * @brief : we get the value of pdf function by this interface
         * @param : the direction of location
         * @return : the value of the pdf function
        *******************************************************************/      
        virtual double value(const vec3& direction) const = 0;//返回方向direction的概率
        
        /*******************************************************************
         * @brief : generate a random number with a Probability model
         * @param :none
         * @return :the Three-dimensional random vector
        *******************************************************************/        
        virtual vec3 generate() const = 0;//返回一个适当分布的概率
};
inline vec3 random_cosine_direction() {
    auto r1 = random_double();
    auto r2 = random_double();
    auto z = sqrt(1-r2);

    auto phi = 2*pi*r1;
    auto x = cos(phi)*sqrt(r2);
    auto y = sin(phi)*sqrt(r2);

    return vec3(x, y, z);
}
inline vec3 random_to_sphere(double radius, double distance_squared) 
{
    auto r1 = random_double();
    auto r2 = random_double();
    auto z = 1 + r2*(sqrt(1-radius*radius/distance_squared) - 1);

    auto phi = 2*pi*r1;
    auto x = cos(phi)*sqrt(1-z*z);
    auto y = sin(phi)*sqrt(1-z*z);

    return vec3(x, y, z);
}

class cosine_pdf : public pdf {
    public:
        cosine_pdf(const vec3& w) { uvw.build_from_w(w); }

        virtual double value(const vec3& direction) const override {
            auto cosine = dot(unit_vector(direction), uvw.w());
            return (cosine <= 0) ? 0 : cosine/pi;
        }

        virtual vec3 generate() const override {
            return uvw.local(random_cosine_direction());
        }

    public:
        onb uvw;
};

class hittable_pdf : public pdf {
    public:
        hittable_pdf(shared_ptr<hittable> p, const point3& origin) : ptr(p), o(origin) {}

        virtual double value(const vec3& direction) const override {
            return ptr->pdf_value(o, direction);
        }

        virtual vec3 generate() const override {
            return ptr->random(o);
        }

    public:
        point3 o;
        shared_ptr<hittable> ptr;
};
class mixture_pdf : public pdf {
    public:
        mixture_pdf(shared_ptr<pdf> p0, shared_ptr<pdf> p1) {
            p[0] = p0;
            p[1] = p1;
        }

        virtual double value(const vec3& direction) const override {
            return 0.5 * p[0]->value(direction) + 0.5 *p[1]->value(direction);
        }

        virtual vec3 generate() const override {
            if (random_double() < 0.5)
                return p[0]->generate();
            else
                return p[1]->generate();
        }

    public:
        shared_ptr<pdf> p[2];
};
#endif