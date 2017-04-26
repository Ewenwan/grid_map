/*
 * GridMapIterator.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

namespace grid_map {

/**
 * @Author      Buyi
 * @DateTime    2017-04-04
 * @version     [version]
 * @function    [GridMapIterator]
 * @description [迭代器构造函数]
 */
GridMapIterator::GridMapIterator(const grid_map::GridMap& gridMap)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  // 返回每行的点乘是什么鬼
  linearSize_ = size_.prod();
  linearIndex_ = 0;
  isPastEnd_ = false;
}

GridMapIterator::GridMapIterator(const GridMapIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  linearSize_ = other->linearSize_;
  linearIndex_ = other->linearIndex_;
  isPastEnd_ = other->isPastEnd_;
}

GridMapIterator& GridMapIterator::operator =(const GridMapIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  linearSize_ = other.linearSize_;
  linearIndex_ = other.linearIndex_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool GridMapIterator::operator !=(const GridMapIterator& other) const
{
  return linearIndex_ != other.linearIndex_;
}

const Index GridMapIterator::operator *() const
{
  return getIndexFromLinearIndex(linearIndex_, size_);
}

const size_t& GridMapIterator::getLinearIndex() const
{
  return linearIndex_;
}

const Index GridMapIterator::getUnwrappedIndex() const
{
  return getIndexFromBufferIndex(*(*this), size_, startIndex_);
}

/**
 * @Author      Buyi
 * @DateTime    2017-04-04
 * @version     [version]
 * @function    [GridMapIterator::operator]
 * @description [++的重载运算符，在迭代的时候使用]
 */
GridMapIterator& GridMapIterator::operator ++()
{
  size_t newIndex = linearIndex_ + 1;
  if (newIndex < linearSize_) 
  {
    linearIndex_ = newIndex;
  }
  else 
  {
    isPastEnd_ = true;
  }
  return *this;
}

GridMapIterator GridMapIterator::end() const
{
  GridMapIterator res(this);
  res.linearIndex_ = linearSize_ - 1;
  return res;
}

/**
 * @Author      Buyi
 * @DateTime    2017-04-04
 * @version     [version]
 * @function    [isPastEnd]
 * @description [迭代器是否超过map范围]
 * @return      [未超过为假，超过为真]
 */
bool GridMapIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace grid_map */
