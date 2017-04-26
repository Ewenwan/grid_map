/*
 * SubmapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

SubmapIterator::SubmapIterator(const grid_map::SubmapGeometry& submap)
    : SubmapIterator(submap.getGridMap(), submap.getStartIndex(), submap.getSize())
{
}

SubmapIterator::SubmapIterator(const grid_map::GridMap& gridMap,
                               const grid_map::BufferRegion& bufferRegion)
    : SubmapIterator(gridMap, bufferRegion.getStartIndex(), bufferRegion.getSize())
{
}
/**
 * (索引坐标中心) (自地图索引坐标中心)
 *     \ ______/_________________
 *     |     |   |              |
 *     |     |子 |              |
 *     |     |地 |    | (实际    |
 *     |     |图 |  __| 坐标中心) |
 *     |     |   |              |
 *     |     |   |              |
 *     |_____|___|______________|
 */

/**
 * @function        [SubmapIterator]
 * @description     [submap的迭代器构造]
 * @param   gridMap             [地图名称]
 * @param   submapStartIndex    [子地图的起始索引值]
 * @param   submapSize          [子地图大小]
 * 可以参考上图
 */
SubmapIterator::SubmapIterator(const grid_map::GridMap& gridMap, const Index& submapStartIndex,
                               const Size& submapSize)
{
  // 实际父地图大小
  size_ = gridMap.getSize ();
  // 实际目的图的索引中心
  startIndex_ = gridMap.getStartIndex ();
  // 子地图的索引中心
  index_ = submapStartIndex;
  submapSize_ = submapSize;
  submapStartIndex_ = submapStartIndex;
  submapIndex_.setZero ();
  isPastEnd_ = false;
}

SubmapIterator::SubmapIterator(const SubmapIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  submapSize_ = other->submapSize_;
  submapStartIndex_ = other->submapStartIndex_;
  index_ = other->index_;
  submapIndex_ = other->submapIndex_;
  isPastEnd_ = other->isPastEnd_;
}

SubmapIterator& SubmapIterator::operator =(const SubmapIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  submapSize_ = other.submapSize_;
  submapStartIndex_ = other.submapStartIndex_;
  index_ = other.index_;
  submapIndex_ = other.submapIndex_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool SubmapIterator::operator !=(const SubmapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& SubmapIterator::operator *() const
{
  return index_;
}

const Index& SubmapIterator::getSubmapIndex() const
{
  return submapIndex_;
}

/**
 * @function    [重载运算符]
 * @desciption  [在迭代的时候增加索引值]
 * @return
 */
SubmapIterator& SubmapIterator::operator ++()
{
  isPastEnd_ = !incrementIndexForSubmap(submapIndex_, index_, submapStartIndex_,
                                        submapSize_, size_, startIndex_);
  return *this;
}

bool SubmapIterator::isPastEnd() const
{
  return isPastEnd_;
}

const Size& SubmapIterator::getSubmapSize() const
{
  return submapSize_;
}

} /* namespace grid_map */

