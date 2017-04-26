/*
 * GridMapPclConverter.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_pcl/GridMapPclConverter.hpp"

namespace grid_map {

GridMapPclConverter::GridMapPclConverter()
{
}

GridMapPclConverter::~GridMapPclConverter()
{
}

/**
 * @function            [initializeFromPolygonMesh]
 * @description         [由网状的PCL点云构造gridmap]
 * @param mesh          [mesh网状格式点云
 *                      http://pointclouds.org/documentation/tutorials/greedy_projection.php]
 * @param resolution    []
 * @param gridMap       []
 * @return
 */
bool GridMapPclConverter::initializeFromPolygonMesh(const pcl::PolygonMesh& mesh,
                                                    const double resolution,
                                                    grid_map::GridMap& gridMap)
{
    // 1.由mesh点云图提取点云
    pcl::PointCloud < pcl::PointXYZ > cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    pcl::PointXYZ minBound;
    pcl::PointXYZ maxBound;
    // 2. 获取点云序列中坐标值最大的的点和坐标值最小的点
    pcl::getMinMax3D(cloud, minBound, maxBound);
    // 3. 设置地图大小，以最大点和最小点构成对角的矩形
    grid_map::Length length = grid_map::Length(maxBound.x - minBound.x, maxBound.y - minBound.y);
    // 4. 设置地图的中心坐标，为矩阵的中心
    grid_map::Position position = grid_map::Position((maxBound.x + minBound.x) / 2.0,
                                                   (maxBound.y + minBound.y) / 2.0);
    // 5. 设置地图属性
    gridMap.setGeometry(length, resolution, position);

    return true;
}

/**
 * @function        [addLayerFromPolygonMesh]
 * @description     [通过mesh添加一层gridmap]
 * @param mesh
 * @param layer
 * @param gridMap
 * @return
 */
bool GridMapPclConverter::addLayerFromPolygonMesh(const pcl::PolygonMesh& mesh,
                                                  const std::string& layer,
                                                  grid_map::GridMap& gridMap)
{
    // 得到一个三维向量(0,0,1)T
    const Eigen::Vector3f ray = -Eigen::Vector3f::UnitZ();

    pcl::PointCloud <pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    pcl::PointXYZ minBound;
    pcl::PointXYZ maxBound;
    pcl::getMinMax3D(cloud, minBound, maxBound);

    gridMap.add(layer);
    int value = 0;

    for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
    {
        const Index index(*iterator);
        grid_map::Position vertexPositionXY;
        gridMap.getPosition(index, vertexPositionXY);

        pcl::PointXYZ point;
        point.x = vertexPositionXY.x();
        point.y = vertexPositionXY.y();
        point.z = maxBound.z + 1.0;

        std::vector<double> candidatePoints;
        for (unsigned i = 0; i < mesh.polygons.size(); ++i)
        {
            pcl::PointXYZ intersectionPoint;
            if (rayTriangleIntersect(point, ray, mesh.polygons[i], cloud, intersectionPoint))
                candidatePoints.push_back(intersectionPoint.z);
        }
        if (candidatePoints.size() > 0)
        {
            gridMap.at(layer, index) =
            *(std::max_element(candidatePoints.begin(), candidatePoints.end()));
        }
        else
        {
            gridMap.at(layer, index) = NAN;
            value++;
        }
    }

    return true;
}

/**
 * @function        [rayTriangleIntersect]
 * @description     []
 * @param point
 * @param ray
 * @param vertices
 * @param pointCloud
 * @param intersectionPoint
 * @return
 */
bool GridMapPclConverter::rayTriangleIntersect(const pcl::PointXYZ& point,
                                               const Eigen::Vector3f& ray,
                                               const pcl::Vertices& vertices,
                                               const pcl::PointCloud<pcl::PointXYZ>& pointCloud,
                                               pcl::PointXYZ& intersectionPoint)
{
  // Algorithm here is adapted from:
  // http://softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm#intersect_RayTriangle()
  //
  // Original copyright notice:
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.

    assert(vertices.vertices.size() == 3);
    /*      参考链接中的公式1
     *  V0：a     P0：p, 可以直接看作是cell正上方的一个点
     *  P1-P0：ray
     *  a，b，c构造了一个平面
     */
    // 根据a，b，c点构造平面，并判断ray与平面的关系
    const Eigen::Vector3f p = point.getVector3fMap();
    const Eigen::Vector3f a = pointCloud[vertices.vertices[0]].getVector3fMap();
    const Eigen::Vector3f b = pointCloud[vertices.vertices[1]].getVector3fMap();
    const Eigen::Vector3f c = pointCloud[vertices.vertices[2]].getVector3fMap();
    const Eigen::Vector3f u = b - a;
    const Eigen::Vector3f v = c - a;
    const Eigen::Vector3f n = u.cross(v);
    const float n_dot_ray = n.dot(ray);

    // 1.1 当分母为0的时候，射线在平面上或者是平行与平面的
    if (std::fabs(n_dot_ray) < 1e-9) return false;

    const float r = n.dot(a - p) / n_dot_ray;

    // 1.2 当r小于0的时候，射线和平面也没有交点
    if (r < 0)
    return false;

    // 判断交点I是否在三角形a，b，c之内，参考公式3
    const Eigen::Vector3f w = p + r * ray - a;
    const float denominator = u.dot(v) * u.dot(v) - u.dot(u) * v.dot(v);
    const float s_numerator = u.dot(v) * w.dot(v) - v.dot(v) * w.dot(u);
    const float s = s_numerator / denominator;
    if (s < 0 || s > 1) return false;

    const float t_numerator = u.dot(v) * w.dot(u) - u.dot(u) * w.dot(v);
    const float t = t_numerator / denominator;
    if (t < 0 || s + t > 1) return false;

    // 最终的交叉点坐标，参考公式4
    Eigen::Vector3f intersecPoint = a + s * u + t * v;

    intersectionPoint.x = intersecPoint.x();
    intersectionPoint.y = intersecPoint.y();
    intersectionPoint.z = intersecPoint.z();

    return true;
}

} /* namespace */
