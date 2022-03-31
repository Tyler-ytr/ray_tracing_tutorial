//material.h
#ifndef MATERIAL_H
#define MATERIAL_H
#include "hittable.h"
#include "texture.h"
class material {
    public:
        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& albedo, ray& scattered, double& pdf
        ) const {
            return false;
        }

        virtual double scattering_pdf(
            const ray& r_in, const hit_record& rec, const ray& scattered
        ) const {
            return 0;
        }

        virtual color emitted(double u, double v, const point3& p) const {
            return color(0,0,0);//默认返回黑色
        }
};

class lambertian:public material{
    public:
        lambertian(const color& a) : albedo(make_shared<solid_color>(a)) {}
        lambertian(shared_ptr<texture> a) : albedo(a) {}
        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& alb, ray& scattered, double& pdf
        ) const override {
            auto scatter_direction = rec.normal + random_unit_vector();

            // Catch degenerate scatter direction
            if (scatter_direction.near_zero())
                scatter_direction = rec.normal;
            scattered = ray(rec.p, unit_vector(scatter_direction), r_in.time());
            alb = albedo->value(rec.u, rec.v, rec.p);
            pdf = dot(rec.normal, scattered.direction()) / pi;
            return true;
        }
        double scattering_pdf(
            const ray& r_in, const hit_record& rec, const ray& scattered
        ) const {
            auto cosine = dot(rec.normal, unit_vector(scattered.direction()));
            return cosine < 0 ? 0 : cosine/pi;
        }

    public:
        shared_ptr<texture> albedo;
};
class metal : public material {
    public:
        metal(const vec3& a, double f) : albedo(a), fuzz(f < 1 ? f : 1) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered
        ) const {
            vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
            scattered = ray(rec.p, reflected + fuzz*random_in_unit_sphere(),r_in.time());
            attenuation = albedo;
            return (dot(scattered.direction(), rec.normal) > 0);//dot<0我们认为吸收
        }

    public:
        vec3 albedo;
        double fuzz;
};

//在可以偏折的情况下总是偏折, 其余情况发生反射的绝缘体材质
class dielectric : public material {
    public:
        dielectric(double ri) : ref_idx(ri) {}
        double schlick(double cosine, double ref_idx) const{
            auto r0 = (1-ref_idx) / (1+ref_idx);
            r0 = r0*r0;
            return r0 + (1-r0)*pow((1 - cosine),5);
        }

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered
        ) const {
            attenuation = vec3(1.0, 1.0, 1.0);
            double etai_over_etat = (rec.front_face) ? (1.0 / ref_idx) : (ref_idx);

            vec3 unit_direction = unit_vector(r_in.direction());
            double cos_theta = ffmin(dot(-unit_direction, rec.normal), 1.0);
            double sin_theta = sqrt(1.0 - cos_theta*cos_theta);
            if (etai_over_etat * sin_theta > 1.0 ) {
                vec3 reflected = reflect(unit_direction, rec.normal);
                scattered = ray(rec.p, reflected,r_in.time());
                return true;
            }
            double reflect_prob = schlick(cos_theta, etai_over_etat);
            if (random_double() < reflect_prob)
            {
                vec3 reflected = reflect(unit_direction, rec.normal);
                scattered = ray(rec.p, reflected,r_in.time());
                return true;
            }
            vec3 refracted = refract(unit_direction, rec.normal, etai_over_etat);
            scattered = ray(rec.p, refracted,r_in.time());
            return true;
        }
    public:
        double ref_idx;
};
class diffuse_light:public material{
    public:
        diffuse_light(shared_ptr<texture> a) : emit(a) {}
        diffuse_light(color c) : emit(make_shared<solid_color>(c)) {}

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered,double& pdf
        ) const override {
            return false;
        }

        virtual color emitted(double u, double v, const point3& p) const override {
            return emit->value(u, v, p);
        }

    public:
        shared_ptr<texture> emit;
};

// //各项同性体积体材质
// class isotropic : public material {
//     public:
//         isotropic(color c) : albedo(make_shared<solid_color>(c)) {}
//         isotropic(shared_ptr<texture> a) : albedo(a) {}

//         virtual bool scatter(
//             const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered
//         ) const override {
//             scattered = ray(rec.p, random_in_unit_sphere(), r_in.time());
//             attenuation = albedo->value(rec.u, rec.v, rec.p);
//             return true;
//         }

//     public:
//         shared_ptr<texture> albedo;
// };


#endif