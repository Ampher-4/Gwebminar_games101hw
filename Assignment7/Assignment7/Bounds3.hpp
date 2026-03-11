//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects

//这他妈是错的，不能单独判断。这玩意儿有一种专门的算法叫做kay-kajiya, 找个抄就行
//        Vector3f pSpatialStorage[] = {pMin, pMax};
//    auto xSol1 = (pSpatialStorage[1 - dirIsNeg[0]].x - ray.origin.x) * invDir.x;
//    auto xSol2 = (pSpatialStorage[dirIsNeg[0]].x - ray.origin.x) * invDir.x;
//    auto ySol1 = (pSpatialStorage[1 - dirIsNeg[1]].y - ray.origin.y) * invDir.y;
//    auto ySol2 = (pSpatialStorage[dirIsNeg[1]].y - ray.origin.y) * invDir.y;
//    auto zSol1 = (pSpatialStorage[1 - dirIsNeg[2]].z - ray.origin.z) * invDir.z;
//    auto zSol2 = (pSpatialStorage[dirIsNeg[2]].z - ray.origin.z) * invDir.z;
//
//    bool x = (xSol1 < xSol2) && (xSol2 > 0);
//    bool y = (ySol1 < ySol2) && (ySol2 > 0);
//    bool z = (zSol1 < zSol2) && (zSol2 > 0);
//    return x && y && z;
// 计算三个轴向的进入和离开时间
    // 根据 dirIsNeg 确定哪一个是 pMin 哪一个是 pMax，保证 t1 < t2
    float t_x_min = (pMin.x - ray.origin.x) * invDir.x;
    float t_x_max = (pMax.x - ray.origin.x) * invDir.x;
    if (dirIsNeg[0]) std::swap(t_x_min, t_x_max); // 如果方向为负，交换 min/max

    float t_y_min = (pMin.y - ray.origin.y) * invDir.y;
    float t_y_max = (pMax.y - ray.origin.y) * invDir.y;
    if (dirIsNeg[1]) std::swap(t_y_min, t_y_max);

    float t_z_min = (pMin.z - ray.origin.z) * invDir.z;
    float t_z_max = (pMax.z - ray.origin.z) * invDir.z;
    if (dirIsNeg[2]) std::swap(t_z_min, t_z_max);

    // 找到所有轴向中 最晚进入 的时间
    float t_enter = std::max(t_x_min, std::max(t_y_min, t_z_min));
    // 找到所有轴向中 最早离开 的时间
    float t_exit  = std::min(t_x_max, std::min(t_y_max, t_z_max));

    // 相交条件：
    // 1. 进入时间必须小于离开时间 (区间没错过)
    // 2. 离开时间必须大于 0 (包围盒不在光线背后)
    return t_enter <= t_exit && t_exit >= 0;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
