/*
 * EllipseIterator.hpp
 *
 *  Created on: Dec 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/EllipseIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

#include <math.h>
#include <Eigen/Geometry>

using namespace std;

namespace grid_map {

/**
 * @function        [EllipseIterator]
 * @desription      [椭圆迭代器的构造函数]
 * @param gridMap   [地图名]
 * @param center    [椭圆中心]
 * @param length    [长短轴的大小]
 * @param rotation  [椭圆的旋转角度，顺时针]
 */
EllipseIterator::EllipseIterator(const GridMap& gridMap, const Position& center, const Length& length, const double rotation)
    : center_(center)
{
    //
    semiAxisSquare_ = (0.5 * length).square();
    // 求取旋转矩阵,为什么这个旋转矩阵是这样？
    double sinRotation = sin(rotation);
    double cosRotation = cos(rotation);
    transformMatrix_ << cosRotation, sinRotation, sinRotation, -cosRotation;
    // 获取父地图的一些信息
    mapLength_ = gridMap.getLength(); 
    mapPosition_ = gridMap.getPosition();
    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    bufferStartIndex_ = gridMap.getStartIndex();

    // 获取椭圆迭代器的起始和终止索引坐标，以及索引区的大小
    Index submapStartIndex;
    Index submapBufferSize;
    findSubmapParameters(center, length, rotation, submapStartIndex, submapBufferSize);
    internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
    if(!isInside()) ++(*this);
}

EllipseIterator& EllipseIterator::operator =(const EllipseIterator& other)
{
    center_ = other.center_;
    semiAxisSquare_ = other.semiAxisSquare_;
    transformMatrix_ = other.transformMatrix_;
    internalIterator_ = other.internalIterator_;
    mapLength_ = other.mapLength_;
    mapPosition_ = other.mapPosition_;
    resolution_ = other.resolution_;
    bufferSize_ = other.bufferSize_;
    bufferStartIndex_ = other.bufferStartIndex_;
    return *this;
}

bool EllipseIterator::operator !=(const EllipseIterator& other) const
{
    return (internalIterator_ != other.internalIterator_);
}

const Eigen::Array2i& EllipseIterator::operator *() const
{
    return *(*internalIterator_);
}

EllipseIterator& EllipseIterator::operator ++()
{
    ++(*internalIterator_);
    if (internalIterator_->isPastEnd())
        return *this;

    for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_))
    {
        if (isInside()) break;
    }

    return *this;
}


bool EllipseIterator::isPastEnd() const
{
    return internalIterator_->isPastEnd();
}

const Size& EllipseIterator::getSubmapSize() const
{
    return internalIterator_->getSubmapSize();
}

/**
 * @function        [isInside]
 * @description     [判断索引是否在椭圆内时候在]
 * @return          []
 */
bool EllipseIterator::isInside() const
{
    Position position;
    // 获取该索引在位置坐标系下的坐标
    getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
    // 同时比较两个轴向上的距离，如果都小在圆内，大则在圆外
    //！x2/a2+y2/b2<=1，则在椭圆内
    double value = ((transformMatrix_ * (position - center_)).array().square() / semiAxisSquare_).sum();
    return (value <= 1);
}

/**
 * @function            [findSubmapParameters]
 * @description         [获取椭圆迭代器的起始和终止索引坐标，以及索引区的大小]
 * @param center        [椭圆圆心]
 * @param length        [长短轴]
 * @param rotation      [旋转角度，顺时针]
 * @param startIndex    []
 * @param bufferSize    []
 */
void EllipseIterator::findSubmapParameters(const Position& center, const Length& length, const double rotation,
                                           Index& startIndex, Size& bufferSize) const
{
    // 1. 初始化旋转矩阵(顺时针旋转)
    //  | cosα  -sinα |
    //  | sinα  cosα  |
    const Eigen::Rotation2Dd rotationMatrix(rotation);
    // 2. 求取旋转后的长轴和短轴
    Eigen::Vector2d u = rotationMatrix * Eigen::Vector2d(length(0), 0.0);
    Eigen::Vector2d v = rotationMatrix * Eigen::Vector2d(0.0, length(1));

    // 3. 构建椭圆的外接矩形
    // 求取短轴和长轴的和向量的长度之和
    const Length boundingBoxHalfLength = (u.cwiseAbs2() + v.cwiseAbs2()).array().sqrt();
    // 求取矩形左上角的点
    Position topLeft = center.array() + boundingBoxHalfLength;
    // 求取矩形右下角的点
    Position bottomRight = center.array() - boundingBoxHalfLength;
    // 4. 限制该position在地图之内
    limitPositionToRange(topLeft, mapLength_, mapPosition_);
    limitPositionToRange(bottomRight, mapLength_, mapPosition_);
    // 5. 获取起始和终止点在索引坐标系下的坐标以及缓冲区的大小。
    getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
    Index endIndex;
    getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
    bufferSize = endIndex - startIndex + Eigen::Array2i::Ones();
}

} /* namespace grid_map */

