/*******************************************************************
 * @Descripttion   : 三角形面片，三棱锥相关
 * @version        : 
 * @Author         : Tyler-ytr
 * @Date           : 2022-04-03 13:07
 * @LastEditTime   : 2022-04-03 14:39
*******************************************************************/
#ifndef TRIANGLE_H
#define TRIANGLE_H
#include "rtweekend.h"

#include "hittable.h"

class triangle:public hittable{
    public:
        triangle(){}
        triangle(point3 _A,point3 _B,point3 _C,shared_ptr<material> mat):A(_A),B(_B),C(_C),mat_ptr(mat){
            
            //AB向量叉乘BC向量
            edge1=B - A;
            edge2=C - A;
            N=unit_vector(cross(edge1,edge2));
            }
        virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;
        // bool is_in_triangle(const point3&p)const;//判断p是否在三角形内部
    private:
        point3 A,B,C;//三角形三条边：A->B,B->C,C->A;从而确定了法向量 AB叉乘AC（等价于AB叉乘BC） 如果是xoz的话天然是往上的
        shared_ptr<material> mat_ptr;
        vec3 N;//法向量
        vec3 edge1,edge2;
};

// bool triangle::is_in_triangle(const point3&p)const{
    
//     //参考了https://www.cnblogs.com/graphics/archive/2010/08/05/1793393.html 使用重心法求解
//     vec3 v0 = C - A;
//     vec3 v1 = B - A;
//     vec3 v2 = p - A;

//     double dot00 = dot(v0, v0);
//     double dot01 = dot(v0, v1);
//     double dot02 = dot(v0, v2);
//     double dot11 = dot(v1, v1);
//     double dot12 = dot(v1, v2);

//     double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);
//     double u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
//     if (u < 0 || u > 1) // if u out of range, return directly
//     {
//         return false ;
//     }
//     double v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
//     if (v < 0 || v > 1) // if v out of range, return directly
//     {
//         return false;
//     }
//     return u + v <= 1;
// }

// Fast, minimum storage ray-triangle intersection.
//http://www.graphics.cornell.edu/pubs/1997/MT97.pdf
//参考https://www.i4k.xyz/article/weixin_43022263/108541782
bool triangle::hit(const ray& r, double tmin, double tmax, hit_record& rec) const{
    const double eps=1e-8;
    const vec3 pvec=cross(r.direction(), edge2);
    const double det=dot(pvec,edge1);
    if(fabs(det)<eps){
        return false;
    }
    const double invdet=1.0/det;
    const vec3 tvec=r.origin()-A;

    const double u=dot(pvec,tvec)*invdet;
    if (u < 0 || u > 1) return false; 
    const vec3 qvec=cross(tvec,edge1);
    const double v= dot(qvec,r.direction())*invdet;

    if(v<0 || u+v>1) return false;

    const double t=dot(qvec,edge2)*invdet;
    if(t<tmin||t>tmax) return false;

    rec.t=t;
    rec.p=r.at(t);
    rec.u=u;
    rec.v=v;
    rec.set_face_normal(r,N);
    rec.mat_ptr = mat_ptr;
    return true;

}
bool triangle::bounding_box(double time0, double time1, aabb& output_box)const{
    point3 minv=point3(std::min(std::min(A.x(),B.x()),C.x()),
                    std::min(std::min(A.y(),B.y()),C.y()),
                    std::min(std::min(A.z(),B.z()),C.z()));
    point3 maxv=point3(std::max(std::max(A.x(),B.x()),C.x()),
                    std::max(std::max(A.y(),B.y()),C.y()),
                    std::max(std::max(A.z(),B.z()),C.z()));
    vec3 diff=maxv-minv;
    maxv.e[0]+=diff.x()<0.001?0.001:0;
    maxv.e[1]+=diff.y()<0.001?0.001:0;
    maxv.e[2]+=diff.z()<0.001?0.001:0;
    output_box=aabb(minv,maxv);
    return true;


}
class pyramid:public hittable{
    public:
        pyramid(){};
        pyramid(point3 _A,point3 _B,point3 _C,point3 _D,shared_ptr<material> mat):A(_A),B(_B),C(_C),D(_D),mat_ptr(mat){
            sides.add(make_shared<triangle>(A,B,C,mat_ptr));
            sides.add(make_shared<triangle>(A,C,D,mat_ptr));
            sides.add(make_shared<triangle>(A,D,B,mat_ptr));
            sides.add(make_shared<triangle>(B,C,D,mat_ptr));
        }
        virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

    private:
        point3 A,B,C,D;//建议A是顶点
        shared_ptr<material> mat_ptr;
        hittable_list sides;
};
bool pyramid::hit(const ray& r, double tmin, double tmax, hit_record& rec)const{
    return sides.hit(r,tmin,tmax,rec);
}
bool pyramid::bounding_box(double time0, double time1, aabb& output_box)const{
    point3 minv=point3(std::min(std::min(std::min(A.x(),B.x()),C.x()),D.x()),
                    std::min(std::min(std::min(A.y(),B.y()),C.y()),D.y()),
                    std::min(std::min(std::min(A.z(),B.z()),C.z()),D.z()));
    point3 maxv=point3(std::max(std::max(std::max(A.x(),B.x()),C.x()),D.x()),
                    std::max(std::max(std::max(A.y(),B.y()),C.y()),D.y()),
                    std::max(std::max(std::max(A.z(),B.z()),C.z()),D.z()));
    vec3 diff=maxv-minv;
    maxv.e[0]+=diff.x()<0.001?0.001:0;
    maxv.e[1]+=diff.y()<0.001?0.001:0;
    maxv.e[2]+=diff.z()<0.001?0.001:0;
    output_box=aabb(minv,maxv);
    return true;
}




#endif