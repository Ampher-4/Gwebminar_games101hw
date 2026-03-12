//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

#include <thread>


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.001;

//void castRayMultiWrapped(const Scene &scene,const Ray &ray, std::vector<Vector3f> &caches, int w, int h, int spp){
//    for(int k = 0; k < spp; k++){
//        caches[h*scene.width + w] += scene.castRay(ray, 0) / spp;
//    }
//}

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    auto threadcache = std::vector<std::thread>();

    // change the spp value to change sample ammount
    extern int polluteSPP;
    int spp = polluteSPP;
    std::cout << "SPP: " << spp << "\n";

    // 获取硬件核心数，通常 i9-12900H 可以开 16-20 个
    int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> workers;

    // 我们可以按行分工
    auto renderRows = [&](uint32_t start_row, uint32_t end_row) {
        for (uint32_t j = start_row; j < end_row; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                Vector3f color(0,0,0);
                for (int k = 0; k < spp; k++) {
                    // 生成射线逻辑...
                    float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale; //165
                    float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale; //699
                    Vector3f dir = normalize(Vector3f(-x, y, 1));
                    color += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
                framebuffer[j * scene.width + i] = color;
            }
        }
    };

    // 分配任务
    uint32_t rows_per_thread = scene.height / num_threads;
    for (int t = 0; t < num_threads; ++t) {
        uint32_t start = t * rows_per_thread;
        uint32_t end = (t == num_threads - 1) ? scene.height : start + rows_per_thread;
        workers.emplace_back(renderRows, start, end);
    }

    // 等待所有工人干完活
    for (auto& t : workers) t.join();

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
