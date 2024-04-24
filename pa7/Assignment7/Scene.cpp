//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
/*
Vector3f Scene::castRay(const Ray &ray, int depth) const
{

    //计算来自光源的贡献+来自反射的贡献
    Vector3f L_dir;
    Vector3f L_indir;

    //intersect(const Ray ray)in Scene.cpp: 求一条光线与场景的交点
    
    //sampleLight(Intersection pos, float pdf)——对光源进行随机采样
    //在场景的所有光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度
    
    //sample(const Vector3f wi, const Vector3f N)
    //按照该材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向

    //pdf(const Vector3f wi, const Vector3f wo, const Vector3f N)
    //给定一对入射、出射方向与法向量，计算 sample 方法得到该出射方向的概率密度

    //eval(const Vector3f wi, const Vector3f wo, const Vector3f N)
    //给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值

    //RussianRoulette：P_RR, Russian Roulette 的概率

    // TO DO Implement Path Tracing Algorithm here
}*/

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // // TO DO Implement Path Tracing Algorithm here
    // Vector3f L_dir;
    // Vector3f L_indir;
 
    // // 从像素发出的光线与物体的交点
    // Intersection obj_inter = intersect(ray);
    // if(!obj_inter.happened)
    //     return L_dir;
 
    // // 打到光源
    // if(obj_inter.m->hasEmission())
    //     return obj_inter.m->getEmission();
 
    // // 打到物体
    // Vector3f p = obj_inter.coords;
    // Material* m = obj_inter.m;
    // Vector3f N = obj_inter.normal.normalized();
    // Vector3f wo = ray.direction; // 像素到物体的向量
    
    // // 有交点，对光源采样
    // float pdf_L = 1.0; //可以不初始化
    // Intersection light_inter ;
    // sampleLight(light_inter,pdf_L);    // 得到光源位置和对光源采样的pdf
    
    // Vector3f x = light_inter.coords;
    // Vector3f ws = (x - p).normalized(); //物体到光源
    // Vector3f NN = light_inter.normal.normalized();  
    // Vector3f emit = light_inter.emit;
    // float d = (x-p).norm();
    
    // // 再次从光源发出一条光线，判断是否能打到该物体，即中间是否有阻挡
    // Ray Obj2Light(p,ws);
    // float d2 = intersect(Obj2Light).distance;
    // // 是否阻挡，利用距离判断，需注意浮点数的处理
    // if(d2-d > -0.001){
    //     Vector3f eval = m->eval(wo,ws,N); // wo不会用到
    //     float cos_theta = dotProduct(N,ws);
    //     float cos_theta_x = dotProduct(NN,-ws);//ws从物体指向光源，与NN的夹角大于180
    //     L_dir = emit * eval * cos_theta * cos_theta_x / std::pow(d,2) / pdf_L;
    // }
    
    // // L_indir
    // float P_RR = get_random_float();
    // if(P_RR<RussianRoulette){
    //     Vector3f wi = m->sample(wo,N).normalized();
    //     Ray r(p,wi);
    //     Intersection inter = intersect(r);
    //     // 判断打到的物体是否会发光取决于m
    //     if(inter.happened && !inter.m->hasEmission()){
    //         Vector3f eval = m->eval(wo,wi,N);
    //         float pdf_O = m->pdf(wo,wi,N);
    //         float cos_theta = dotProduct(wi,N);
    //         L_indir = castRay(r, depth+1) * eval * cos_theta/ pdf_O / RussianRoulette;
    //     }
    // }
    // //4->16min
    // return L_dir + L_indir;

     // TO DO Implement Path Tracing Algorithm here
    // 获取光线交点
    Intersection intersection = intersect(ray);
    // 没有碰到物体的情况
    if(!intersection.happened)
        return Vector3f();
  
    // 射到光源的情况
    if(intersection.m->hasEmission())
        return intersection.m->getEmission();
  
    // 射到物体的情况
    Vector3f L_dir, L_indir;

    // 先对光源采样
    Intersection inter_light;
    float pdf_light = 0.f;
    sampleLight(inter_light, pdf_light);
    Vector3f obj2light = inter_light.coords - intersection.coords;
    Vector3f ws = obj2light.normalized();
    float dis = obj2light.norm();
  
    // 如果射到的物体比光源距离近就说明被挡住了
    if(intersect(Ray(intersection.coords, ws)).distance - dis >= -EPSILON ) 
        L_dir = inter_light.emit * intersection.m->eval(ray.direction, ws, intersection.normal) * dotProduct(ws, intersection.normal) * dotProduct(-ws, inter_light.normal) / (dis*dis) / pdf_light;
  
    // 俄罗斯轮盘赌
    // Manually specify a probability P_RR
    // Randomly select ksi in a uniform dist. in [0, 1]
    // If (ksi > P_RR) return 0.0;
    if(get_random_float() > RussianRoulette)
        return L_dir;
  
    Vector3f wi = intersection.m->sample(ray.direction, intersection.normal).normalized();

    Ray nextObjRay(intersection.coords, wi);
    Intersection nextInter = intersect(nextObjRay);
    if(nextInter.happened && !nextInter.m->hasEmission()) 
        L_indir = castRay(nextObjRay, depth+1) * intersection.m->eval(ray.direction, wi, intersection.normal) * dotProduct(wi, intersection.normal) / intersection.m->pdf(ray.direction, wi, intersection.normal) / RussianRoulette;
  
    return L_dir + L_indir;
}