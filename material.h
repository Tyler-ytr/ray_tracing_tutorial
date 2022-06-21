/*
 * @Author: your name
 * @Date: 2022-04-01 17:04:28
 * @LastEditTime: 2022-04-01 18:01:11
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /raytracing/ray_tracing_tutorial/material.h
 */
//material.h
#ifndef MATERIAL_H
#define MATERIAL_H
#include "hittable.h"
#include "texture.h"
#include "onb.h"
#include "pdf.h"
struct scatter_record {
    ray specular_ray;
    bool is_specular;
    color attenuation;//实际上表示混合之后的颜色
    shared_ptr<pdf> pdf_ptr;
};

class material {
    public:
        virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v,
            const point3& p) const {

                return color(0,0,0);
        }

        virtual bool scatter(
            const ray& r_in, const hit_record& rec, scatter_record& srec
        ) const {
            return false;
        }

        virtual double scattering_pdf(
            const ray& r_in, const hit_record& rec, const ray& scattered
        ) const {
            return 0;
        }
};
inline bool double_near_zero(double test){
    double dis=1e-6;
    return abs(test)<dis;//如果等于0，返回true；否则false

}
class lambertian:public material{
    public:
        lambertian(const color& a) : albedo(make_shared<solid_color>(a)) {}
        lambertian(shared_ptr<texture> a) : albedo(a) {}
        virtual bool scatter(
            const ray& r_in, const hit_record& rec, scatter_record& srec
        ) const override {
            srec.is_specular = false;
            srec.attenuation = albedo->value(rec.u, rec.v, rec.p);
            srec.pdf_ptr = make_shared<cosine_pdf>(rec.normal);
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
            const ray& r_in, const hit_record& rec, scatter_record& srec
        ) const override {
            vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
            srec.specular_ray = ray(rec.p, reflected+fuzz*random_in_unit_sphere());
            srec.attenuation = albedo;
            srec.is_specular = true;
            srec.pdf_ptr = 0;
            return true;
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
            const ray& r_in, const hit_record& rec, scatter_record& srec
        ) const {
            srec.is_specular = true;
            srec.pdf_ptr = nullptr;
            srec.attenuation = color(1.0, 1.0, 1.0);
            double etai_over_etat = (rec.front_face) ? (1.0 / ref_idx) : (ref_idx);

            vec3 unit_direction = unit_vector(r_in.direction());
            double cos_theta = ffmin(dot(-unit_direction, rec.normal), 1.0);
            double sin_theta = sqrt(1.0 - cos_theta*cos_theta);
            if (etai_over_etat * sin_theta > 1.0 ) {
                vec3 reflected = reflect(unit_direction, rec.normal);
                srec.specular_ray = ray(rec.p, reflected,r_in.time());
                return true;
            }
            double reflect_prob = schlick(cos_theta, etai_over_etat);
            if (random_double() < reflect_prob)
            {
                vec3 reflected = reflect(unit_direction, rec.normal);
                srec.specular_ray = ray(rec.p, reflected,r_in.time());
                return true;
            }
            vec3 refracted = refract(unit_direction, rec.normal, etai_over_etat);
            srec.specular_ray = ray(rec.p, refracted,r_in.time());
            return true;
        }
    public:
        double ref_idx;
};
class diffuse_light:public material{
    public:
        diffuse_light(shared_ptr<texture> a) : emit(a) {}
        diffuse_light(color c) : emit(make_shared<solid_color>(c)) {}


        virtual color emitted(const ray& r_in, const hit_record& rec, double u, double v,
            const point3& p) const override {

            if (rec.front_face)
                return emit->value(u, v, p);
            else
                return color(0,0,0);
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
class Microfast :public material{
    public:
        Microfast(const color& a,const double _roughness,const double _ior):
            Diffuse_Color(make_shared<solid_color>(a)),roughness(_roughness),ior(_ior){};
        Microfast(shared_ptr<texture> a,const double _roughness,const double _ior):
            Diffuse_Color(a),roughness(_roughness),ior(_ior){};
        virtual bool scatter(
            const ray& r_in, const hit_record& rec, scatter_record& srec
        ) const override {
            
            srec.is_specular = false;
            srec.attenuation = Diffuse_Color->value(rec.u, rec.v, rec.p);
            srec.pdf_ptr = make_shared<cosine_pdf>(rec.normal);
            return true;

        }

        double Fresnel_Schlick(double cosine)const {
            double F0=abs ((1.0 - ior) / (1.0 + ior));
            return F0 + (1-F0) * pow( 1 -  cosine, 5);
        }
        double GGX_Partial(double x_dot_m,double x_dot_n)const {
            if( x_dot_m/x_dot_n<0)return 0;
            double x_dot_m2=x_dot_m*x_dot_m;
            double tan2=(1-x_dot_m2)/x_dot_m2;
            return 2.0/(1+sqrt(1+roughness*roughness*tan2));
        }
        double GGX(double o_dot_n,double o_dot_m,double i_dot_n,double i_dot_m)const{
            return GGX_Partial(o_dot_m,o_dot_n)*GGX_Partial(i_dot_m,i_dot_n);
        }
        double Distribution(double m_dot_n)const{
            if(m_dot_n-1e-6<=0.0)return 0.0;
            double m_dot_n2=m_dot_n*m_dot_n;
            double den=m_dot_n2*roughness*roughness+(1-m_dot_n2);
            return roughness*roughness/(pi*den*den);

        }


        double scattering_pdf(
            const ray& r_in, const hit_record& rec, const ray& scattered
        ) const {
            //Cook-Torrance 里面有两部分 一部分是cosine/pi;另一部分是DFG/(4*(w_0\cdot n)*(w_0\cdot n))
            //第一部分
            double result=0.0;
            auto cosine = dot(rec.normal, unit_vector(scattered.direction()));
            result+=cosine < 0 ? 0 : cosine/pi;

            //第二部分

            double eps=1e-6;
            //和GAMES101里面的符号一致
            vec3 wi=unit_vector(-r_in.dir);
            vec3 wo=unit_vector(scattered.dir);
            vec3 n=rec.normal;
            vec3 m=unit_vector(wi+wo);

            double o_dot_n=std::max(0.0, dot(wo, n));
            double o_dot_m =std::max(0.0, dot(wo, m));
            double i_dot_n=std::max(0.0, dot(wi, n));
            double i_dot_m =std::max(0.0, dot(wi, m));
            double m_dot_n =std::max(0.0, dot(m, n));
            if (o_dot_n<=0)return result;

            //double m=roughness;
            //菲涅尔项计算
            double F=Fresnel_Schlick(i_dot_m);  
            double G=GGX(o_dot_n,o_dot_m,i_dot_n,i_dot_m);
            double D=Distribution(m_dot_n);
            result+=F*G*D/(4*i_dot_n);
            return result;
        }
    public:
        shared_ptr<texture> Diffuse_Color;// Diffuse Color
        double roughness;//粗糙度
        double ior;//折射指数(Indices of Refraction)
};

#endif