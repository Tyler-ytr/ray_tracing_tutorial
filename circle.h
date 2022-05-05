/*******************************************************************
 * @Descripttion   : 圆形光源相关
 * @version        : 
 * @Author         : Tyler-ytr
 * @Date           : 2022-04-03 09:51
 * @LastEditTime   : 2022-05-05 16:09
*******************************************************************/
#ifndef CIRCLE_H
#define CIRCLE_H
#include "rtweekend.h"

#include "hittable.h"
#include "sample.h"
class circle: public hittable{
    public:
        circle():sampler(10,2){randomlist=sampler.sampling();sample_cnt=0; sample_size=randomlist.size();}
        circle(vec3 c,vec3 f,double r,shared_ptr<material> mat):center(c),further_point(f),R(r),mat_ptr(mat),sampler(10,2){N=unit_vector(further_point-center);randomlist=sampler.sampling();sample_cnt=0;sample_size=randomlist.size();}
        virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;

        // virtual double pdf_value(const point3& origin, const vec3& v) const override{
        //     hit_record rec;
        //     if (!this->hit(ray(origin, v), 0.001, infinity, rec))
        //         return 0;
        //     double area = R*R*pi;//相当于公式里面的dA
        //     double distance_squared = rec.t * rec.t * v.length_squared();//相当于分母的dis
        //     double cosine = fabs(dot(v, rec.normal) / v.length());
        //     return distance_squared / (cosine * area);
        // }
        // virtual vec3 random(const point3& origin) const override{
        //     double r = sqrt(random_double());//0-1随机数
        //     double theta=random_double(0, 2*pi);//随机角度
        //     //只考虑xoz的情况
        //     // vec3 randomPoint = vec3(center.x() + r * cos(theta), center.y(), center.z() + r * sin(theta));
        //     // return randomPoint -origin;
        //     //不止考虑xoz的情况
        //     //生成两个垂直于法向量的单位向量构成坐标系来解决
        //     vec3 OP_unit=vertical_unit_vector(N);
        //     vec3 OQ_unit=cross(OP_unit,N);
        //     vec3 randompoint=center+r*cos(theta)*OP_unit+r*sin(theta)*OQ_unit;
        //     return randompoint-origin;
            
        // }
        virtual double pdf_value(const point3& origin, const vec3& v) const override{
            hit_record rec;
            if (!this->hit(ray(origin, v), 0.001, infinity, rec))
                return 0;
            double area = R*R*pi;//相当于公式里面的dA
            double distance_squared = rec.t * rec.t * v.length_squared();//相当于分母的dis
            double cosine = fabs(dot(v, rec.normal) / v.length());
            return distance_squared / (cosine * area);
        }
        virtual vec3 random(const point3& origin)  override{
            sample_cnt=sample_cnt+1;
            if(sample_cnt==sample_size){
                re_sample();
            }
            
            double r = sqrt(randomlist[sample_cnt].first);//0-1随机数
            double theta=2*pi*randomlist[sample_cnt].second;//随机角度
            //只考虑xoz的情况
            // vec3 randomPoint = vec3(center.x() + r * cos(theta), center.y(), center.z() + r * sin(theta));
            // return randomPoint -origin;
            //不止考虑xoz的情况
            //生成两个垂直于法向量的单位向量构成坐标系来解决
            vec3 OP_unit=vertical_unit_vector(N);
            vec3 OQ_unit=cross(OP_unit,N);
            vec3 randompoint=center+r*cos(theta)*OP_unit+r*sin(theta)*OQ_unit;
            return randompoint-origin;
            
        }
        void re_sample(){
            randomlist=sampler.sampling();
            sample_cnt=0;
            sample_size=randomlist.size();
        }
    public:
        vec3 center;
        double R;
        shared_ptr<material> mat_ptr;
        point3 further_point;//远处的点，center-->further_point是法相,所以如果是xz_circle的话应该是(0,-1,0),指向下面
        vec3 N;//法相
        Sampler sampler;
        std::vector<std::pair<double,double>> randomlist;
        int sample_cnt;
        int sample_size;

};
bool circle::hit(const ray& r, double tmin, double tmax, hit_record& rec) const{
    const double eps=1e-8;
    vec3 aMinusP0 = r.origin() - center;
    vec3 n=N;
    double a=dot(aMinusP0,n);
    double b=dot(r.direction(),n);
    if(fabs(b)<eps){
        return false;
    }

    double tempT=-a / b;
    vec3 p=r.at(tempT);
	if (dot(p - center, p - center) > R * R)
		return false;
    if (tempT < tmax && tempT > tmin){
        rec.t=tempT;
        rec.p=r.at(rec.t);

        rec.set_face_normal(r,n);

        rec.mat_ptr = mat_ptr;
        return true;
    }
    return false;
    
}


bool circle::bounding_box(double time0, double time1, aabb& output_box) const{
    //四个点求包围盒
    std::vector<vec3> points;
    //首先求底面，定P,Q让O1P垂直于O1Q
    vec3 OP_unit=vertical_unit_vector(N);
    vec3 OQ_unit=cross(OP_unit,N);
    vec3 OP=OP_unit*(R+0.01);
    vec3 OQ=OQ_unit*(R+0.01);
    points.push_back(OP-OQ+center);
    points.push_back(OP+OQ+center);
    points.push_back(-OP+OQ+center);
    points.push_back(-OP-OQ+center);

    double minx=infinity;
    double miny=infinity;
    double minz=infinity;

    double maxx=-infinity;
    double maxy=-infinity;
    double maxz=-infinity;
    for(int i=0;i<points.size();i++){
        if(points[i].x()<minx){
            minx=points[i].x();
        }
        if(points[i].x()>maxx){
            maxx=points[i].x();
        }
        if(points[i].y()<miny){
            miny=points[i].y();
        }
        if(points[i].y()>maxy){
            maxy=points[i].y();
        }
        if(points[i].z()<minz){
            minz=points[i].z();
        }
        if(points[i].z()>maxz){
            maxz=points[i].z();
        }
    }
    output_box = aabb(point3(minx,miny,minz),point3(maxx,maxy,maxz));
    return true;

}




#endif