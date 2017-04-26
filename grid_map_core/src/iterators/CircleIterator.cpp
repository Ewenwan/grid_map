/*
 * Circleterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/CircleIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

/**
 * @function        [CircleIterator]
 * @description     [圆形迭代器的构造函数]
 * @param gridMap   [父地图]
 * @param center    [圆形子地图的圆心]
 * @param radius    [圆形子地图的半径]
 */
CircleIterator::CircleIterator(const GridMap& gridMap, const Position& center, const double radius)
    : center_(center),
      radius_(radius)
{
    // 获取关于父地图的一些信息，包括大小、分辨率、位置
    radiusSquare_ = pow(radius_, 2);
    mapLength_ = gridMap.getLength();
    mapPosition_ = gridMap.getPosition();
    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    bufferStartIndex_ = gridMap.getStartIndex();
    Index submapStartIndex;
    Index submapBufferSize;
    //
    findSubmapParameters(center, radius, submapStartIndex, submapBufferSize);
    internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
    // 如果刚开始不在圆内，则迭代一次
    if(!isInside()) ++(*this);
}

CircleIterator& CircleIterator::operator =(const CircleIterator& other)
{
  center_ = other.center_;
  radius_ = other.radius_;
  radiusSquare_ = other.radiusSquare_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool CircleIterator::operator !=(const CircleIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& CircleIterator::operator *() const
{
  return *(*internalIterator_);
}

/**
 * @function        [operator ++]
 * @description     [cycle的迭代运算]
 * @return
 */
CircleIterator& CircleIterator::operator ++()
{
    // 迭代加一次
    ++(*internalIterator_);
    if (internalIterator_->isPastEnd())
        return *this;

    // 确保每次迭代后的索引都要在圆内，在第一次迭代之后，判断在不在圆内，如果不在圆内，则继续迭代，直至迭代点在圆内为止

    for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_))
    {
        if (isInside()) break;
    }
    // 根据submapSize_的大小，最终的迭代终止点应该是bottomRight
    return *this;
}

bool CircleIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

/**
 * @function        [isInside]
 * @description     [判断该索引处的坐标是否超过圆的范围]
 * @return
 */
bool CircleIterator::isInside() const
{
  // 后去该索引值下的位置坐标
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  // 通过比较半径的平方，判断该索引点是否在圆内。
  double squareNorm = (position - center_).array().square().sum();
  return (squareNorm <= radiusSquare_);
}

/**
 * @function    [findSubmapParameters]
 * @description [设置圆形子地图(外接矩形)的参数]
 * @param       center      [圆形子地图的圆心]
 * @param       radius      [半径]
 * @param       startIndex  [索引的其实位置]
 * @param       bufferSize  [地图大小]
 * @return                     [true]
 */
void CircleIterator::findSubmapParameters(const Position& center, const double radius,
                                          Index& startIndex, Size& bufferSize) const
{
  // 1. 求取最左上角和右上角的两个点,相当于外界矩形
  Position topLeft = center.array() + radius;
  Position bottomRight = center.array() - radius;
  // 2. 确保两个点在map的范围之内
  limitPositionToRange(topLeft, mapLength_, mapPosition_);
  limitPositionToRange(bottomRight, mapLength_, mapPosition_);
  // 3. 获取左右侧两个点在父地图下的索引值
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  // 4.这样计算圆形map的方法对么？
  bufferSize = endIndex - startIndex + Index::Ones();
}

} /* namespace grid_map */

