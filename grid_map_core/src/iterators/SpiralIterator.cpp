/*
 * SpiralIterator.hpp
 *
 *  Created on: Jul 7, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/SpiralIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

#include <cmath>

using namespace std;

namespace grid_map {

/**
 * @function        [SpiralIterator]
 * @description     [构造旋转迭代的圆形迭代器]
 * @param gridMap   [父地图]
 * @param center    [圆形地图圆心]
 * @param radius    [圆形地图半径]
 */
SpiralIterator::SpiralIterator(const grid_map::GridMap& gridMap, const Eigen::Vector2d& center,
                               const double radius)
    : center_(center),
      radius_(radius),
      distance_(0)
{
    // 获取父地图的信息，包括大小、坐标和分辨率
    radiusSquare_ = radius_ * radius_;
    mapLength_ = gridMap.getLength();
    mapPosition_ = gridMap.getPosition();
    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    // 获取圆心在索引坐标系下的索引坐标
    gridMap.getIndex(center_, indexCenter_);
    // 求取不小于radius_ / resolution_的整数
    // 也就是索引坐标系下的半径
    nRings_ = std::ceil(radius_ / resolution_);
    // 判断刚开始的索引点是否在地图内
    if (checkIfIndexWithinRange(indexCenter_, bufferSize_))
        pointsRing_.push_back(indexCenter_);
    else
        generateRing();
}

SpiralIterator& SpiralIterator::operator =(const SpiralIterator& other)
{
  center_ = other.center_;
  indexCenter_ = other.indexCenter_;
  radius_ = other.radius_;
  radiusSquare_ = other.radiusSquare_;
  nRings_ = other.nRings_;
  distance_ = other.distance_;
  pointsRing_ = other.pointsRing_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  return *this;
}

bool SpiralIterator::operator !=(const SpiralIterator& other) const
{
  return (pointsRing_.back() != pointsRing_.back()).any();
}

const Eigen::Array2i& SpiralIterator::operator *() const
{
  return pointsRing_.back();
}

/**
 * @function        [++]
 * @description     [迭代重载运算符]
 * @return
 */
SpiralIterator& SpiralIterator::operator ++()
{
    // 先把最后一个index给删了，相当于return一次弹一次
    pointsRing_.pop_back();
    // 当这个环空的时候，再增加环
    if (pointsRing_.empty() && !isPastEnd())
        generateRing();
    return *this;
}

bool SpiralIterator::isPastEnd() const
{
    return (distance_ == nRings_ && pointsRing_.empty());
}

/**
 * @function        [isInside]
 * @description     [还是通过距离来判断该点是否在圆内]
 * @param index     [索引坐标系下的索引值]
 * @return
 */
bool SpiralIterator::isInside(const Index index) const
{
    // 1. 通过索引坐标获取位置坐标
    Eigen::Vector2d position;
    getPositionFromIndex(position, index, mapLength_, mapPosition_, resolution_, bufferSize_);
    // 2. 计算该点到圆心的距离
    double squareNorm = (position - center_).array().square().sum();
    return (squareNorm <= radiusSquare_);
}

/**
 * @function        [generateRing]
 * @description     [获取距圆心距离为distance的环上的点
 *                  按照顺时针的顺序，绕圆心，以distance_为半径，求取圆上的点存放到pointsRing_中]
 *
 * 
 */
void SpiralIterator::generateRing()
{
    distance_++;
    Index point(distance_, 0);
    Index pointInMap;
    Index normal;
    do
    {
        // 1.增加mappoint点
        pointInMap.x() = point.x() + indexCenter_.x();
        pointInMap.y() = point.y() + indexCenter_.y();
        // 判断增加了坐标值的点是否还在map内
        if (checkIfIndexWithinRange(pointInMap, bufferSize_))
        {
            // 当距离值等于或者接近半径时，要判断该点是否还在圆内
            if (distance_ == nRings_ || distance_ == nRings_ - 1)
            {
                if (isInside(pointInMap))
                    pointsRing_.push_back(pointInMap);
            }
            else
            {
                pointsRing_.push_back(pointInMap);
            }
        }
        // 2. 螺旋式递增算法
        // 2.1 获取下一个迭代点的方向：0，不动  1，向右或者向下  -1，向左或者向下
        normal.x() = -signum(point.y());
        normal.y() = signum(point.x());

        // 2.1 在x轴上判断，移动后的点是否满足要求
        if (normal.x() != 0
            && (int) Vector(point.x() + normal.x(), point.y()).norm() == distance_)
            point.x() += normal.x();
        // 2.2 在y轴上判断，移动后的点是否满足要求
        else if (normal.y() != 0
            && (int) Vector(point.x(), point.y() + normal.y()).norm() == distance_)
            point.y() += normal.y();
        // 2.3 如果上面两个条件都不满足，则在两个轴上同时移动
        else
        {
            point.x() += normal.x();
            point.y() += normal.y();
        }
    // 当且仅当point.x()等于当前距离且y的值等于0的时候跳出
    } while (point.x() != distance_ || point.y() != 0);
}

double SpiralIterator::getCurrentRadius() const
{
  Index radius = *(*this) - indexCenter_;
  return radius.matrix().norm() * resolution_;
}

} /* namespace grid_map */

