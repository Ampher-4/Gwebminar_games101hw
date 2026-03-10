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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
//    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;


    if(intersection.happened) {
        //so you hit a point. now consider dir light and in-dir light with RR probability.

        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal of the hit point surface
        Vector2f st; // st coordinates (uv actually)
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st); //???????
//        Vector3f tmp = hitPoint;

        //Directional light contribution
        Intersection dirlightIntersect = {}; float dirlightPdf = 0.0f;
        sampleLight(dirlightIntersect, dirlightPdf);
        Intersection dirlightResult = Scene::intersect(Ray{hitPoint, dirlightIntersect.coords - hitPoint});
        Vector3f L_dir = {0.0f};
        if(dirlightResult.happened && ((dirlightResult.coords - hitPoint).norm() - (dirlightIntersect.coords - hitPoint).norm()) < EPSILON){
            
            //note: in this implementation we use fernel term as brdf. 
            auto L_i = (dirlightIntersect.coords - hitPoint).normalized(); //points to the light source
            L_dir = L_i * m->eval(L_i, -ray.direction, N) * dotProduct(L_i, N) * dotProduct(-L_i, dirlightIntersect.normal) / (L_i.norm() * L_i.norm()) / dirlightPdf;
        }else{
            L_dir = 0.0f;
        }


        // Indirect light contribution (with Russian Roulette)
        Vector3f L_indir = {0.0f};
        if (get_random_float() > RussianRoulette) {
//            L_indir = 0.0f;
            L_indir;
        } else {
            auto wi = m->sample(hitPoint, N);
            wi = normalize(wi);
            Intersection indirLightIntersect = Scene::intersect(Ray{hitPoint, wi});

            //if hit and object is not light source
            if(indirLightIntersect.happened && !indirLightIntersect.obj->hasEmit()){
                L_indir = castRay(Ray{hitPoint, wi}, depth + 1) * m->eval(wi, -ray.direction, N) * dotProduct(wi, N) / m->pdf(wi, -ray.direction, N) / RussianRoulette;
            }

        }

        hitColor = L_dir + L_indir;
    }

    return hitColor;
}