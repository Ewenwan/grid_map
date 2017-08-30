/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/LineIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

/**
 * @function        [LineIterator]
 * @description     [线性迭代器的构造函数，用位置坐标系下的坐标构造
 *                  先判断了该位置坐标是否在map之内，
 *                  然后进行初始化工作]
 * @param gridMap   [地图名]
 * @param start     [起始坐标点]
 * @param end       [终止坐标点]
 */
LineIterator::LineIterator(const grid_map::GridMap& gridMap, const Position& start,
                           const Position& end)
{
    Index startIndex, endIndex;
    if (getIndexLimitedToMapRange(gridMap, start, end, startIndex)
    && getIndexLimitedToMapRange(gridMap, end, start, endIndex))
        initialize(gridMap, startIndex, endIndex);
}

    /**
 * @function        [LineIterator]
 * @description     [线性迭代器的构造函数，用索引坐标系下的坐标构造
 *                  因为索引坐标系是循环的，所以不需要判断该点是否在图之内]
 * @param gridMap   [地图名]
 * @param start     [起始坐标点]
 * @param end       [终止坐标点]
 */
LineIterator::LineIterator(const grid_map::GridMap& gridMap, const Index& start, const Index& end)
{
    initialize(gridMap, start, end);
}

LineIterator& LineIterator::operator =(const LineIterator& other)
{
    index_ = other.index_;
    start_ = other.start_;
    end_ = other.end_;
    iCell_ = other.iCell_;
    nCells_ = other.nCells_;
    increment1_ = other.increment1_;
    increment2_ = other.increment2_;
    denominator_ = other.denominator_;
    numerator_ = other.numerator_;
    numeratorAdd_ = other.numeratorAdd_;
    mapLength_ = other.mapLength_;
    mapPosition_ = other.mapPosition_;
    resolution_ = other.resolution_;
    bufferSize_ = other.bufferSize_;
    bufferStartIndex_ = other.bufferStartIndex_;
    return *this;
}

bool LineIterator::operator !=(const LineIterator& other) const
{
    return (index_ != other.index_).any();
}

const Index& LineIterator::operator *() const
{
    return index_;
}

/**
 * https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
 * https://zh.wikipedia.org/zh-cn/%E5%B8%83%E9%9B%B7%E6%A3%AE%E6%BC%A2%E5%A7%86%E7%9B%B4%E7%B7%9A%E6%BC%94%E7%AE%97%E6%B3%95
 * @function        [operator ++]
 * @description     [重载运算符，属于Bresenham画线算法的核心部分]
 * @return          [index]
 *
 *      numerator_： dx/2       denominator_：dx     numeratorAdd_：dy
 *      原始伪代码：
 *      error = error + dy/dx     (error初始值为0)
 *      If ( error >= 0.5)
 *          y = y + 1
 *          error = error -1
 *      x = x + 1
 *
 *      本代码：
 *      error = error + dx/2 + dy
 *      numerator_ >= denominator_ ==>  dx/2 + dy >=dx   ==>  dy/dx >=0.5
 *      具体可以参考wiki种最佳化的解决方法。
 *      

 */
LineIterator& LineIterator::operator ++()
{
    numerator_ += numeratorAdd_;  // Increase the numerator by the top of the fraction
    if (numerator_ >= denominator_)
    {
        // 在下一次判断的时候还是一样的
        // dx/2 + dy -dx >dx ==> dy/dx - 1 > 0.5
        numerator_ -= denominator_;
        index_ += increment1_;
    }
    index_ += increment2_;
    ++iCell_;
    return *this;
}

bool LineIterator::isPastEnd() const
{
    // 总的索引个数
    return iCell_ >= nCells_;
}

/**
 * @function        [initialize]
 * @description     [线性迭代器的初始化函数]
 * @param gridMap   [地图名]
 * @param start     [起始索引]
 * @param end       [终止索引]
 * @return          []
 */
bool LineIterator::initialize(const grid_map::GridMap& gridMap, const Index& start, const Index& end)
{
    // 1. 获取一些地图信息
    start_ = start;
    end_ = end;
    mapLength_ = gridMap.getLength();
    mapPosition_ = gridMap.getPosition();
    resolution_ = gridMap.getResolution();
    bufferSize_ = gridMap.getSize();
    bufferStartIndex_ = gridMap.getStartIndex();
    Index submapStartIndex;
    Size submapBufferSize;
    // 2. 初始化迭代器参数
    initializeIterationParameters();
    return true;
}

/**
 * @function        []
 * @description     []
 * @param gridMap   []
 * @param start     []
 * @param end       []
 * @param index     []
 * @return
 */
bool LineIterator::getIndexLimitedToMapRange(const grid_map::GridMap& gridMap,
                                             const Position& start, const Position& end,
                                             Index& index)
{
    // 1. 求取直线方向
    Position newStart = start;
    Vector direction = (end - start).normalized();
    // 2. 检测start是否在map之内
    while (!gridMap.getIndex(newStart, index))
    {
        // 2.1 如果起始点start不在map内，则让start沿着end点移动一个方向分辨率点
        newStart += (gridMap.getResolution() - std::numeric_limits<double>::epsilon()) * direction;
        // 如果移动之后起始点start和终止点end之间距离不足一个分辨率，则初始化失败
        if ((end - newStart).norm() < gridMap.getResolution() - std::numeric_limits<double>::epsilon())
            return false;
    }
    // 2.1 如果在，则返回真值
    return true;
}

/**
 * @function        [initializeIterationParameters]
 * @description     [初始化迭代器参数
 *                  这个地方主要是针对直线在坐标系下的8种情况(4个象限，2种斜率)做的不同的画线策略]
 *                  [参数的说明可以参考上面的伪代码处解释]
 */
void LineIterator::initializeIterationParameters()
{
    iCell_ = 0;
    index_ = start_;

    Size delta = (end_ - start_).abs();
    // 自左向右
    if (end_.x() >= start_.x())
    {
        // x-values increasing.
        increment1_.x() = 1;
        increment2_.x() = 1;
    }
    // 自右向左
    else
    {
        // x-values decreasing.
        increment1_.x() = -1;
        increment2_.x() = -1;
    }

    // 自下向上
    if (end_.y() >= start_.y())
    {
        // y-values increasing.
        increment1_.y() = 1;
        increment2_.y() = 1;
    }
    // 自上向下
    else
    {
        // y-values decreasing.
        increment1_.y() = -1;
        increment2_.y() = -1;
    }

    // 斜率小于1
    if (delta.x() >= delta.y())
    {
        //！因为x>y,所以一个y至少对应一个x，所以cell的个数可以按照x轴统计
        // There is at least one x-value for every y-value.
        increment1_.x() = 0; // Do not change the x when numerator >= denominator.
        increment2_.y() = 0; // Do not change the y for every iteration.
        denominator_ = delta.x();
        numerator_ = delta.x() / 2;
        numeratorAdd_ = delta.y();
        nCells_ = delta.x() + 1; // There are more x-values than y-values.
    }
    // 斜率大于1
    // 这种情况下，相当于把直线沿y=x对称即可，把y，x交换处理。参考wiki种的一般化方法。
    else
    {
        // There is at least one y-value for every x-value
        increment2_.x() = 0; // Do not change the x for every iteration.
        increment1_.y() = 0; // Do not change the y when numerator >= denominator.
        denominator_ = delta.y();
        numerator_ = delta.y() / 2;
        numeratorAdd_ = delta.x();
        nCells_ = delta.y() + 1; // There are more y-values than x-values.
    }
}

} /* namespace grid_map */
