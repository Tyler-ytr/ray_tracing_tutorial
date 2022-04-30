/*******************************************************************
 * @Descripttion   : 采样
 * @version        : 
 * @Author         : Tyler-ytr
 * @Date           : 2022-04-30 21:29
 * @LastEditTime   : 2022-04-30 22:17
*******************************************************************/

#ifndef SAMPLE_H
#define SAMPLE_H

#include "rtweekend.h"
#include <string>

// 参考 https://www.bilibili.com/read/cv11405334 以及论文Fast Poisson Disk Sampling in Arbitrary Dimensions
class Fastpoisson{
    private:
        int width;
        int height;
        double threshold;
        int max_attempts;

    public:
        Fastpoisson():width(100),height(100),threshold(6),max_attempts(50) {};
        Fastpoisson(const int _width,const int _height,const double _threshold,const int _max_attempts):width(100),height(100),threshold(6),max_attempts(_max_attempts) {};

        std::vector<std::pair<double,double>> Fastpoissonsampling();
        
};
/*******************************************************************
 * @brief : 在r-2r的圆环上面随机生成一点
 * @param :doubleundefined r
 * @return :*undefined
*******************************************************************/
inline std::pair<double,double> random_in_cirque(double r){
    double x=random_double(r,2*r);
    double y=random_double(sqrt(x*x-r*r),sqrt(4*r*r-x*x));
    if(random_double()<0.5){
        x=-x;
    }
    if(random_double()<0.5){
        y=-y;
    }
    return std::make_pair(x,y);
}
std::vector<std::pair<double,double>>Fastpoisson::Fastpoissonsampling(){
    //step 0
    std::vector<std::pair<double,double>> result_list;//存储采样的点的vec
    double cell_size=threshold/std::sqrt(2);//小格子的大小，threthold相当于小格子的斜边
    int cols=std::ceil(width/cell_size);//列数
    int rows=std::ceil(height/cell_size);//行数
    // std::cout<<cols<<" "<<rows<<std::endl<<"----------!!!!!!!"<<std::endl;
    
    //用来标记小格子是否被采样过的二维矩阵，初始化为-1；
    std::vector<std::vector<int>> grids(rows,std::vector<int>(cols,-1));
    double r=threshold;

    //step 1
    std::pair<double,double> start=std::make_pair(random_double(0,width),random_double(0,height));//起始点
    
    int col=std::floor(start.first/cell_size);//起始点所在的列
    int row=std::floor(start.second/cell_size);//起始点所在的行
    int start_key=result_list.size();//起始点的key,应该是0
    grids[row][col]=start_key;//起始点被标记为start_key
    // std::cout<<start.first<<std::endl;
    std::vector<int>active_list;
    active_list.push_back(start_key);//初始化”active list"（候选点的集合）
    result_list.push_back(start);
    //step 2
    int cnt=0;
    while(active_list.size()>0){
        //当候选点的集合非空的时候，从里面选择一个随机的索引
        // int key=0;
        int key=active_list[random_int(0,active_list.size()-1)];
        std::pair<double,double>point=result_list[key];//选择的点
        //生成最多k个点
        bool found=false;
        for(int i=0;i<max_attempts;++i){

            std::pair<double,double> random_point= random_in_cirque(r);//生成一个[r, 2r)圆环上的随机点
            // 给原有的采样点 point 加上一个距离 [r, 2r) 的随机向量，成为新的采样点
            std::pair<double,double> new_point=std::make_pair(point.first+random_point.first,point.second+random_point.second);

            if(new_point.first<0||new_point.first>=width||new_point.second<0||new_point.second>=height){
                continue;
            }
            
            col=std::floor(new_point.first/cell_size);
            row=std::floor(new_point.second/cell_size);
            // std::cout<<new_point.first<<" "<<new_point.second<<std::endl;
            // std::cout<<cell_size<<" "<<cell_size<<std::endl;
            // std::cout<<col<<" "<<row<<std::endl;
            // std::cout<<grids.size()<<" "<<grids[0].size()<<std::endl;
            // std::cout<<"----------!\n";
        
            if(grids[row][col]!=-1){//已经存在这个点了
                continue;
            }
            //检查采样点周围区块是否存在距离小于threshold的点
            bool ok=true;
            int min_c=std::floor((new_point.first-threshold)/cell_size);
            int min_r=std::floor((new_point.second-threshold)/cell_size);
            int max_c=std::floor((new_point.first+threshold)/cell_size);
            int max_r=std::floor((new_point.second+threshold)/cell_size);
            
            [&](){
                for(int r=min_r;r<=max_r;++r){
                    if(r<0||r>=rows)
                        continue;
                    for(int c=min_c;c<=max_c;++c){
                        if(c<0||c>=cols)
                            continue;
                        int point_key=grids[r][c];
                        if(point_key!=-1){
                            std::pair<double,double> round_point=result_list[point_key];
                            double distance=std::sqrt(std::pow(new_point.first-round_point.first,2)+std::pow(new_point.second-round_point.second,2));
                            if(distance<threshold){
                                ok=false;
                                return;
                            }
                            // 当 ok 为 false 后，后续的循环检测都没有意义的了，
                            // 使用 return 跳出两层循环。 
                        }
                    }
                }
            }();
            
            if(ok){
                int new_point_key=result_list.size();
                result_list.push_back(new_point);
                grids[row][col]=new_point_key;
                active_list.push_back(new_point_key);
                found=true;
                break;
            }  
            
        }
        if(!found){
            //如果没有找到，则从active list中删除这个点
            auto iter=std::find(active_list.begin(),active_list.end(),key);
            if(iter!=active_list.end()){
                active_list.erase(iter);
            }
        }
    }
    return result_list;
};


//sampling
/*******************************************************************
 * @brief : 输入采样类型，两个点之间的间隔（相对于100*100的图），输出采样点的集合（x,y都是0-1之间，做好正则化处理）
 * @param :*undefined
 * @return :*undefined
*******************************************************************/
class Sampler{
    private:
        double threshold;//两个点之间的间隔（对于100*100的图,比如10的时候对于uniform sampling横坐标就是{5,15,25,...,95}）
        int Sampler_type;//0:random 1:uniform 2:Fastpoisson
        
    public:
        Sampler():threshold(6),Sampler_type(0){};
        Sampler(double _threshold,int _Sampler_type):threshold(_threshold),Sampler_type(_Sampler_type){};
        std::vector<std::pair<double,double>> sampling();

};

#endif