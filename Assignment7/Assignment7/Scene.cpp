//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <cmath>


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

    Intersection SamplePosition = Scene::intersect(ray);
    Material *m = SamplePosition.m;
    Object *hitObject = SamplePosition.obj;
    Vector3f hitColor = Vector3f{0.0f};
//    float tnear = kInfinity;
    Vector2f uv;
    uint32_t index = 0;


    if(SamplePosition.happened) {
        //so you hit a point. now consider dir light and in-dir light with RR probability.

        //well if you lucky enough to hit a light source, just return the emission of the light source.
        if(hitObject->hasEmit()){
            return depth == 0 ? m->getEmission() : Vector3f{0.0f};
        }

        Vector3f hitPoint = SamplePosition.coords;
        Vector3f hitPointBias = hitPoint + EPSILON * SamplePosition.normal; 
        // avoid self-intersection, offset the original along hitpoint surface normal
        Vector3f N = SamplePosition.normal; // normal of the hit point surface
//        Vector2f st; // st coordinates (uv actually)
//        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st); //???????
//        Vector3f tmp = hitPoint;

        //Directional light contribution
        Intersection DirlightSamplePosition = {}; float dirlightPdf = 0.0f;
        sampleLight(DirlightSamplePosition, dirlightPdf);


        Intersection TestDirLight = Scene::intersect(Ray{hitPointBias, (DirlightSamplePosition.coords - hitPoint).normalized()});


        Vector3f L_dir = {0.0f};
        if(TestDirLight.happened && (std::abs(TestDirLight.distance - (DirlightSamplePosition.coords - hitPointBias).norm()) <= EPSILON )){
            
            //note: in this implementation we use fernel term as brdf. 
            auto L_i = (DirlightSamplePosition.coords - hitPoint).normalized(); //points to the light source 
            auto r = (TestDirLight.coords - hitPoint).norm();
            float cosTheta = fmax(dotProduct(L_i, N), 0.0f);
            float cosThetaPrime = fmax(dotProduct(-L_i, DirlightSamplePosition.normal), 0.0f);

            L_dir = DirlightSamplePosition.emit * m->eval(L_i, -ray.direction, N) * cosTheta * cosThetaPrime / (r * r) / dirlightPdf;
        }else{
            ;
        }


        // Indirect light contribution (with Russian Roulette)
        Vector3f L_indir = {0.0f};
        if (get_random_float() < RussianRoulette) {
            auto wi = m->sample(hitPoint, N);//in diffuse , first param is not used. it just return a random direction in hemisphere
            wi = normalize(wi);
            Intersection indirLightIntersect = Scene::intersect(Ray{hitPoint, wi});

            //if hit and object is not light source
            if(indirLightIntersect.happened && !indirLightIntersect.obj->hasEmit()){
                float cosTheta = fmax(dotProduct(wi, N), 0.0f);

                L_indir = castRay(Ray{hitPointBias, wi}, depth + 1) * m->eval(wi, -ray.direction, N) * cosTheta / m->pdf(wi, (-ray.direction).normalized(), N) / RussianRoulette;
            }

        }

        hitColor = L_dir + L_indir;
    }

    return hitColor;
}