/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/PolygonIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

/**
 * @function        [PolygonIterator]
 * @description     [多边形迭代器构造函数]
 * @param gridMap   [地图名称]
 * @param polygon   [要添加的多边形]
 */
PolygonIterator::PolygonIterator(const grid_map::GridMap& gridMap, const grid_map::Polygon& polygon)
    : polygon_(polygon)
{
    // 获取父地图的一些参数信息
    mapLength_ = gridMap.getLength();
    mapPosition_ = gridMap.getPosition();
    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    bufferStartIndex_ = gridMap.getStartIndex();
    //获取多边形迭代器的起始迭代点和索引区的大小
    Index submapStartIndex;
    Size submapBufferSize;
    findSubmapParameters(polygon, submapStartIndex, submapBufferSize);
    internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
    if(!isInside()) ++(*this);
}

PolygonIterator& PolygonIterator::operator =(const PolygonIterator& other)
{
  polygon_ = other.polygon_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool PolygonIterator::operator !=(const PolygonIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& PolygonIterator::operator *() const
{
  return *(*internalIterator_);
}

/**
 * @description     [迭代器的迭代重载函数,可以参考圆迭代器]
 * @return
 */
PolygonIterator& PolygonIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) break;
  }

  return *this;
}

bool PolygonIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

/**
 * @descrption      [判断该索引对应的坐标位置是否在多边形内]
 * @return
 */
bool PolygonIterator::isInside() const
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  return polygon_.isInside(position);
}

/**
 * @function        [findSubmapParameters]
 * @descrption      [获取多边形迭代器的起始迭代点和索引区的大小]
 * @param polygon   [要迭代的多边形]
 * @param startIndex    [起始索引]
 * @param bufferSize    [索引区大小]
 */
void PolygonIterator::findSubmapParameters(const grid_map::Polygon& polygon, Index& startIndex, Size& bufferSize) const
{
    // 1. 实际是要做多边形的外接矩形
    // 求取了x轴的最大值，y轴的最大值，然后组了一个topleft点
    // 求取了x轴的最大值，y轴的最大值，然后组了一个bottomright点
    Position topLeft = polygon_.getVertices()[0];
    Position bottomRight = topLeft;
    for (const auto& vertex : polygon_.getVertices())
    {
        topLeft = topLeft.array().max(vertex.array());
        bottomRight = bottomRight.array().min(vertex.array());
    }
    // 2. 限制该position在地图之内
    limitPositionToRange(topLeft, mapLength_, mapPosition_);
    limitPositionToRange(bottomRight, mapLength_, mapPosition_);
    // 3. 获取起始和终止点的索引，以及索引区的大小
    getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
    Index endIndex;
    getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
    bufferSize = getSubmapSizeFromCornerIndeces(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace grid_map */

