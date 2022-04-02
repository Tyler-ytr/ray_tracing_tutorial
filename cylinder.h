/*******************************************************************
 * @Descripttion   : 圆柱体
 * @version        : 
 * @Author         : tyler-ytr
 * @Date           : 2022-04-01 21:15
 * @LastEditTime   : 2022-04-02 17:01
*******************************************************************/
//cylinder.h
#ifndef CYLINDER_H
#define CYLINDER_H
#include "hittable.h"
#include "vec3.h"

class cylinder: public hittable{
    public:

        cylinder(){};
        cylinder(vec3 _O1,vec3 _O2,double _R,shared_ptr<material> m):O1(_O1),O2(_O2),R(_R), mat_ptr(m){};        
        virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
        virtual bool bounding_box(double time0, double time1, aabb& output_box) const override;
        bool hitatside(const ray& r, double tmin, double tmax, hit_record& rec)const;
        bool hitatbase(const ray& r, double tmin, double tmax, vec3 O,hit_record& rec)const;
        
    private:
        vec3 O1,O2;//底面圆心，顶面圆心
        double R;//半径
        shared_ptr<material> mat_ptr;
        void get_sphere_uv(const point3& c, double& u, double& v){
            u = 0.5, v = 0.5;
            vec3 vertical=unit_vector(O2-O1);//圆柱一个向上的方向
            vec3 horizontal=vertical_unit_vector(vertical);//一个水平的方向
            


        }

};
/*******************************************************************
 * @brief : 圆柱体的hit函数，基本逻辑是：圆柱体分成三个部分：上下底面以及侧面，光线与三者分别求交点。与侧面的交点是光线和侧面的方程联立可以得到t，然后交点和圆柱体在那个轴的圆形切面的圆心连起来可以得到法向量，进而得到切面求出射光线。最后三个交点找最小的t以及对应的光线即可。
 * @param :ray&undefined 入射的光线
 * @param :doubleundefined tmin t的最小值
 * @param :doubleundefined tmax t的最大值
 * @param :hit_record&undefined rec 命中的记录
 * @return :返回是否命中
*******************************************************************/
bool  cylinder:: hitatside(const ray& r, double tmin, double tmax, hit_record& rec)const{
    const double eps=1e-8;
    vec3 ray_v=r.direction();
    vec3 ray_o=r.origin();
    vec3 N1=unit_vector(O1-O2);//指向底面外部（相当于公式里面的-N1）
    vec3 N2=unit_vector(O2-O1);//指向顶面外部（相当于公式里面的-N2）

    vec3 VtimesN=cross(ray_v,(-N1));
    vec3 O1OtimesN=cross(ray_o-O1,(-N1));
    double a=dot(VtimesN,VtimesN);
    double b=2*dot(VtimesN,O1OtimesN);
    double c=dot(O1OtimesN,O1OtimesN)-R*R;
    double delta=b*b-4*a*c;//二次方程

    double T=0.0;
    //bool front_face=false;//如果是在外部相交为true

    if(delta>eps){
        //有两个解
        delta=sqrt(delta);
        //t1<t2
        double t1=(-b-delta)/(2*a);
        double t2=(-b+delta)/(2*a);

        if(t2<eps){
            return false;
        }else{
            if(t1<=eps){
                if(t2< tmax && t2>tmin){
                    //t1<0<t2说明圆柱侧面和射线在正方向上只有一个交点，并且交点是在圆柱侧面的内部
                    T = t2;
                    //front_face=false;
                    //判断是否在圆柱高度范围里面
                    vec3 C=r.at(T);
                    vec3 O1C=C-O1;
                    vec3 O2C=C-O2;
                    double O1C_N1=dot(O1C,N1);double O2C_N2=dot(O2C,N2);//N1N2指向圆柱的外部，所以结果大于0是不在圆柱里面
                    if(O1C_N1>eps||O2C_N2>eps){
                        return false;
                    }
                }
                else
                {return false;}
            }else{
                //0<t1<t2说明可能有两个交点，需要检查是否在圆柱里面
                bool t1check=false;
                bool t2check=false;
                t1check=(t1>tmin&&t1<tmax);
                t2check=(t2>tmin&&t2<tmax);//确保结果在tmin和tmax之间
                if(t1check||t2check){
                    //判断O1C和N的点积，这里如果都<0说明是合法的（因为N是向外面的）
                    vec3 C1=r.at(t1);
                    vec3 O1C1=C1-O1;
                    vec3 O2C1=C1-O2;
                    double O1C1_N1=dot(O1C1,N1);
                    double O2C1_N2=dot(O2C1,N2);

                    vec3 C2=r.at(t2);
                    vec3 O1C2=C2-O1;
                    vec3 O2C2=C2-O2;
                    double O1C2_N1=dot(O1C2,N1);
                    double O2C2_N2=dot(O2C2,N2);

                    if(t1check&&O1C1_N1<eps&&O2C1_N2<eps){
                        T=t1;
                        //front_face=true;
                    }else if(t2check&&O1C2_N1<eps&&O2C2_N2<eps){
                        T=t2;
                        //front_face=true;
                    }else{
                        return false;
                    }
                }else{
                    return false;
                }
            }
        }
        
    }else{
        return false;
    }
    rec.t=T;
    // rec.front_face=front_face;
    rec.p = r.at(rec.t);
    vec3 outward_normal=unit_vector((rec.p-O1)-dot((rec.p-O1),-N1)*(-N1));
    //if(front_face==false)outward_normal=-outward_normal;
    rec.set_face_normal(r, outward_normal);//设置法向量

    rec.mat_ptr = mat_ptr;
    return true;
}
bool  cylinder:: hitatbase(const ray& r, double tmin, double tmax, vec3 O,hit_record& rec)const{
    const double eps=1e-8;
    vec3 ray_v=r.direction();
    vec3 ray_o=r.origin();
    vec3 N;//向外部的法向量
    
    if((O-O1).near_zero()){
        N=unit_vector(O1-O2);//圆柱底面单位法向量指向底面的外部
    }else{
        N=unit_vector(O2-O1);//圆柱顶面单位法向量指向顶面的外部
    }
    
    //判断是否平行，也就是看法向量和光线的方向的内积是否为0：
    double d=dot(ray_v,N);
    if(abs(d)<eps){
        return false;
    }
    double OO1_N=dot(ray_o-O,N);//如果小于零说明交点在射线的负方向,假设ray_O为O，圆心为O1
    if(OO1_N<eps){
        return false;
    }
    double t=OO1_N/d;
    if (t < tmax && t > tmin){
        vec3 C=r.at(t);
        vec3 O1C=C-O;
        double O1C_lens=O1C.length_squared();
        if(O1C_lens-R*R>eps){
            return false;
        }

        rec.t=t;
        rec.p=C;

        rec.set_face_normal(r,N);
        rec.mat_ptr = mat_ptr;
        return true;
    }
    return false;
}   


bool  cylinder::hit(const ray& r, double tmin, double tmax, hit_record& rec) const{
    hit_record reclist[3]; 
    // rec1,rec2,rec3;
    bool hitside=hitatside(r,tmin,tmax,reclist[0]);//侧面
    bool hitbase1=hitatbase(r,tmin,tmax,O1,reclist[1]);//底面
    bool hitbase2=hitatbase(r,tmin,tmax,O2,reclist[2]);//顶面
    double minT=tmax;
    if(hitside||hitbase1||hitbase2){
        for(int i=0;i<3;i++){
            if(reclist[i].t<minT){
                minT=reclist[i].t;
                rec=reclist[i];
            }
        }
        return true;
    }else{
        return false;
    }


    


}
bool cylinder::bounding_box(double time0, double time1, aabb& output_box) const{
    return false;
}

#endif