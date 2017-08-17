/*
 * GridMapMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMapMath.hpp"

// fabs
#include <cmath>

// Limits
#include <limits>

using namespace std;

namespace grid_map {

namespace internal {

/*!
 * Gets the vector from the center of the map to the origin
 * of the map data structure.
 * @param[out] vectorToOrigin the vector from the center of the map the origin of the map data structure.
 * @param[in] mapLength the lengths in x and y direction.
 * @return true if successful.
 */
/**

 * @function    [getVectorToOrigin]
 * @description [获取地图中心的坐标]
 * @param       vectorToOrigin [中心点坐标]
 * @param       mapLength      [地图大小]
 * @return                     [true]
 */
inline bool getVectorToOrigin(Vector& vectorToOrigin, const Length& mapLength)
{
  // 将向量以矩阵的形式表达
  vectorToOrigin = (0.5 * mapLength).matrix();
  return true;
}


/*!
 * Gets the vector from the center of the map to the center
 * of the first cell of the map data.
 * @param[out] vectorToFirstCell the vector from the center of the cell to the center of the map.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
/**
 * @function    [getVectorToFirstCell]
 * @description [计算第一个索引cell中心点到map中心点的vector]
 * @param       vectorToFirstCell [第一个cell的坐标位置]
 * @param       mapLength         [地图大小]
 * @param       resolution        [地图分辨率]
 * @return                        [true]
 */
inline bool getVectorToFirstCell(Vector& vectorToFirstCell,
                                 const Length& mapLength, const double& resolution)
{
  Vector vectorToOrigin;
  // 1. 获取地图中心坐标(m)
  // vectorToOrigin = (0.5 * mapLength).matrix()
  getVectorToOrigin(vectorToOrigin, mapLength);

  // 2. 计算到中心cell的vector
  // Vector to center of cell.
  // vectorToFirstCell = ((0.5 * mapLength) - (0.5 * resolution)).matrix
  vectorToFirstCell = (vectorToOrigin.array() - 0.5 * resolution).matrix();
  return true;
}

/**
 * @function    [getBufferOrderToMapFrameTransformation]
 * @description [返回一个负的单位阵
 *              为什么是负的？]
 * @return      [description]
 */
inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
{
  return -Eigen::Matrix2i::Identity();
}

/**
 * @function    [getMapFrameToBufferOrderTransformation]
 * @description [返回一个负的单位阵]
 * @return      [description]
 */
inline Eigen::Matrix2i getMapFrameToBufferOrderTransformation()
{
  return getBufferOrderToMapFrameTransformation().transpose();
}

inline bool checkIfStartIndexAtDefaultPosition(const Index& bufferStartIndex)
{
  return ((bufferStartIndex == 0).all());
}

/**
 * @function    [getBufferIndexFromIndex]
 * @description [获取buffer中的索引值]
 * @param       index            [description]
 * @param       bufferSize       [description]
 * @param       bufferStartIndex [description]
 * @return                       [description]
 */
inline Index getBufferIndexFromIndex(
    const Index& index,
    const Size& bufferSize,
    const Index& bufferStartIndex)
{
    if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return index;

    Index bufferIndex = index + bufferStartIndex;
    // 确保bufferIndex在缓冲区buffer之内
    mapIndexWithinRange(bufferIndex, bufferSize);
    return bufferIndex;
}

/**
 * @function    []
 * @description []
 * @param       index            [description]
 * @param       bufferSize       [缓冲区(索引区大小)]
 * @param       bufferStartIndex [开始索引的索引值]
 * @return                       [description]
 */
inline Vector getIndexVectorFromIndex(
    const Index& index,
    const Size& bufferSize,
    const Index& bufferStartIndex)
{
    Index unwrappedIndex;
    // 1. 得到展开的index，不是循环的index。
    // 其实就是在计算当前索引值到开始索引值的坐标差(或者是xy轴方向的cell个数)
    unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);
    return (getBufferOrderToMapFrameTransformation() * unwrappedIndex.matrix()).cast<double>();
}

/**
 * @function    []
 * @description []
 * @param       indexVector      [description]
 * @param       bufferSize       [description]
 * @param       bufferStartIndex [description]
 * @return                       [description]
 */
inline Index getIndexFromIndexVector(
    const Vector& indexVector,
    const Size& bufferSize,
    const Index& bufferStartIndex)
{
    // 1. 做强制类型转换，由double转int
    Index index = (getMapFrameToBufferOrderTransformation() * indexVector.cast<int>()).array();
    return getBufferIndexFromIndex(index, bufferSize, bufferStartIndex);
}

inline BufferRegion::Quadrant getQuadrant(const Index& index, const Index& bufferStartIndex)
{
  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::Quadrant::TopLeft;
  if (index[0] >= bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return BufferRegion::Quadrant::TopRight;
  if (index[0] <  bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return BufferRegion::Quadrant::BottomLeft;
  if (index[0] <  bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return BufferRegion::Quadrant::BottomRight;
}

} // namespace

using namespace internal;

/**
 * @function    [getPositionFromIndex]
 * @description [根据cell的index获取cell的位置(m)]
 * @param       position         [最后获取到cell的位置坐标(x,y)(m)]
 * @param       index            [要获取坐标cell的索引]
 * @param       mapLength        [地图的大小]
 * @param       mapPosition      [地图的位置]
 * @param       resolution       [地图的分辨率]
 * @param       bufferSize       [缓冲区大小]
 * @param       bufferStartIndex [开始搜索的索引值]
 * @return                       [成功为真，否则为假]
 */
bool getPositionFromIndex(Position& position,
                          const Index& index,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const double& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex)
{
    // 1.判断索引是否满足要求(从数值角度和buffer大小角度判断)
    if (!checkIfIndexWithinRange(index, bufferSize)) return false;
    Vector offset;
    // 2. 获取地图中心的位置
    // offset = ((0.5 * mapLength) - (0.5 * resolution)).matrix
    getVectorToFirstCell(offset, mapLength, resolution);
    // 3. 计算索引的position
    // position = 地图坐标 + 地图中原点坐标 + 分辨率*cell个数
    position = mapPosition + offset + resolution * getIndexVectorFromIndex(index, bufferSize, bufferStartIndex);
    return true;
}

/**
 * @function    [getIndexFromPosition]
 * @description [获取positon处的索引值
 *              这个地方实际就是一个坐标系的转换，由距离的坐标系(图的中心为坐标原点，标称值单位m)和索引时坐标系(左上角为原点，标称值为cell单元格)的一个转换]
 * @param       index            [description]
 * @param       position         [description]
 * @param       mapLength        [description]
 * @param       mapPosition      [description]
 * @param       resolution       [description]
 * @param       bufferSize       [description]
 * @param       bufferStartIndex [description]
 * @return                       [description]
 */
bool getIndexFromPosition(Index& index,
                          const Position& position,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const double& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex)
{
    // 1. 判断position是否在map内
     if (!checkIfPositionWithinMap(position, mapLength, mapPosition)) return false;
    Vector offset;
    // 2. 获取地图中心坐标
    // offset = (0.5 * mapLength).matrix()
    getVectorToOrigin(offset, mapLength);
    // 3. 获取索引值
    Vector indexVector = ((position - offset - mapPosition).array() / resolution).matrix();
    index = getIndexFromIndexVector(indexVector, bufferSize, bufferStartIndex);
    return true;
}

/**
 * @function    [checkIfPositionWithinMap]
 * @description [检查该坐标是否在map内]
 * @param       position    [坐标位置]
 * @param       mapLength   [地图大小]
 * @param       mapPosition [地图坐标]
 * @return                  [在为真，不在为假]
 */
bool checkIfPositionWithinMap(const Position& position,
                              const Length& mapLength,
                              const Position& mapPosition)
{
    Vector offset;
    // 获取地图中心的坐标
    // offset = (0.5 * mapLength).matrix()
    getVectorToOrigin(offset, mapLength);
    Position positionTransformed = getMapFrameToBufferOrderTransformation().cast<double>() * (position - mapPosition - offset);

    if (positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0
      && positionTransformed.x() < mapLength(0) && positionTransformed.y() < mapLength(1)) 
    {
        return true;
    }
    return false;
}

void getPositionOfDataStructureOrigin(const Position& position,
                                      const Length& mapLength,
                                      Position& positionOfOrigin)
{
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);
  positionOfOrigin = position + vectorToOrigin;
}

bool getIndexShiftFromPositionShift(Index& indexShift,
                                    const Vector& positionShift,
                                    const double& resolution)
{
  Vector indexShiftVectorTemp = (positionShift.array() / resolution).matrix();
  Eigen::Vector2i indexShiftVector;

  for (int i = 0; i < indexShiftVector.size(); i++) {
    indexShiftVector[i] = static_cast<int>(indexShiftVectorTemp[i] + 0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
  }

  indexShift = (getMapFrameToBufferOrderTransformation() * indexShiftVector).array();
  return true;
}

bool getPositionShiftFromIndexShift(Vector& positionShift,
                                    const Index& indexShift,
                                    const double& resolution)
{
  positionShift = (getBufferOrderToMapFrameTransformation() * indexShift.matrix()).cast<double>() * resolution;
  return true;
}

/**
 * @function    [checkIfIndexWithinRange]
 * @description [判断index是否满足要求：符号是否正确、是否在索引区之内]
 * @param       index      [索引值]
 * @param       bufferSize [缓冲区大小]
 * @return                 [满足要求为真，否则为假]
 */
bool checkIfIndexWithinRange(const Index& index, const Size& bufferSize)
{
  if (index[0] >= 0 && index[1] >= 0 && index[0] < bufferSize[0] && index[1] < bufferSize[1])
  {
    return true;
  }
  return false;
}

/**
 * @function    [mapIndexWithinRange]
 * @description [如果索引值的大小超过了缓冲区的大小，则将其取余索引值大小，返回到循环区内]
 * @param       index      [索引值]
 * @param       bufferSize [索引区大小]
 */
void mapIndexWithinRange(Index& index, const Size& bufferSize)
{
    for (int i = 0; i < index.size(); i++) 
    {
        mapIndexWithinRange(index[i], bufferSize[i]);
    }
}

/**
 * @function    [mapIndexWithinRange]
 * @description [如果索引值不在索引区内，再将其变换到索引区内：
 *               1.如果索引值为负值，则取( buffize - |index|)
 *               2.如果索引值大于buffize,则直接取buffize的余数
 *              ]
 * @param       index      [description]
 * @param       bufferSize [description]
 */
void mapIndexWithinRange(int& index, const int& bufferSize)
{
    // 对应description1
    if (index < 0) 
    {
        index += ((-index / bufferSize) + 1) * bufferSize;
    }
    // 对应description2
    index = index % bufferSize;
}

/**
 * @function    [limitPositionToRange]
 * @description [限制确保该position必须在map之内]
 * @param       position      [位置坐标]
 * @param       mapLength     [地图的长宽大小]
 * @param       mapPosition   [地图位置]
 */
void limitPositionToRange(Position& position, const Length& mapLength, const Position& mapPosition)
{
    // 1. 计算当前位置距离地图左侧的位置差
    Vector vectorToOrigin;
    getVectorToOrigin(vectorToOrigin, mapLength);
    Position positionShifted = position - mapPosition + vectorToOrigin;

    //
    // We have to make sure to stay inside the map.
    for (int i = 0; i < positionShifted.size(); i++)
    {
        double epsilon = 10.0 * numeric_limits<double>::epsilon(); // TODO Why is the factor 10 necessary.
        if (std::fabs(position(i)) > 1.0)
            epsilon *= std::fabs(position(i));

        if (positionShifted(i) <= 0)
        {
            positionShifted(i) = epsilon;
            continue;
        }
        if (positionShifted(i) >= mapLength(i))
        {
            positionShifted(i) = mapLength(i) - epsilon;
            continue;
        }
    }

    position = positionShifted + mapPosition - vectorToOrigin;
}

const Eigen::Matrix2i getBufferOrderToMapFrameAlignment()
{
  return getBufferOrderToMapFrameTransformation().array().abs().matrix();
}

bool getSubmapInformation(Index& submapTopLeftIndex,
                          Size& submapBufferSize,
                          Position& submapPosition,
                          Length& submapLength,
                          Index& requestedIndexInSubmap,
                          const Position& requestedSubmapPosition,
                          const Length& requestedSubmapLength,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const double& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex)
{
  // (Top left / bottom right corresponds to the position in the matrix, not the map frame)
  Eigen::Matrix2d transform = getMapFrameToBufferOrderTransformation().cast<double>();

  // Corners of submap.
  Position topLeftPosition = requestedSubmapPosition - transform * 0.5 * requestedSubmapLength.matrix();
  limitPositionToRange(topLeftPosition, mapLength, mapPosition);
  if(!getIndexFromPosition(submapTopLeftIndex, topLeftPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex)) return false;
  Index topLeftIndex;
  topLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);

  Position bottomRightPosition = requestedSubmapPosition + transform * 0.5 * requestedSubmapLength.matrix();
  limitPositionToRange(bottomRightPosition, mapLength, mapPosition);
  Index bottomRightIndex;
  if(!getIndexFromPosition(bottomRightIndex, bottomRightPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex)) return false;
  bottomRightIndex = getIndexFromBufferIndex(bottomRightIndex, bufferSize, bufferStartIndex);

  // Get the position of the top left corner of the generated submap.
  Position topLeftCorner;
  if(!getPositionFromIndex(topLeftCorner, submapTopLeftIndex, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex)) return false;
  topLeftCorner -= transform * Position::Constant(0.5 * resolution);

  // Size of submap.
  submapBufferSize = bottomRightIndex - topLeftIndex + Index::Ones();

  // Length of the submap.
  submapLength = submapBufferSize.cast<double>() * resolution;

  // Position of submap.
  Vector vectorToSubmapOrigin;
  getVectorToOrigin(vectorToSubmapOrigin, submapLength);
  submapPosition = topLeftCorner - vectorToSubmapOrigin;

  // Get the index of the cell which corresponds the requested
  // position of the submap.
  if(!getIndexFromPosition(requestedIndexInSubmap, requestedSubmapPosition, submapLength, submapPosition, resolution, submapBufferSize)) return false;

  return true;
}

Size getSubmapSizeFromCornerIndeces(const Index& topLeftIndex, const Index& bottomRightIndex,
                                    const Size& bufferSize, const Index& bufferStartIndex)
{
  const Index unwrappedTopLeftIndex = getIndexFromBufferIndex(topLeftIndex, bufferSize, bufferStartIndex);
  const Index unwrappedBottomRightIndex = getIndexFromBufferIndex(bottomRightIndex, bufferSize, bufferStartIndex);
  return Size(unwrappedBottomRightIndex - unwrappedTopLeftIndex + Size::Ones());
}

bool getBufferRegionsForSubmap(std::vector<BufferRegion>& submapBufferRegions,
                               const Index& submapIndex,
                               const Size& submapBufferSize,
                               const Size& bufferSize,
                               const Index& bufferStartIndex)
{
  if ((getIndexFromBufferIndex(submapIndex, bufferSize, bufferStartIndex) + submapBufferSize > bufferSize).any()) return false;

  submapBufferRegions.clear();

  Index bottomRightIndex = submapIndex + submapBufferSize - Index::Ones();
  mapIndexWithinRange(bottomRightIndex, bufferSize);

  BufferRegion::Quadrant quadrantOfTopLeft = getQuadrant(submapIndex, bufferStartIndex);
  BufferRegion::Quadrant quadrantOfBottomRight = getQuadrant(bottomRightIndex, bufferStartIndex);

  if (quadrantOfTopLeft == BufferRegion::Quadrant::TopLeft) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopLeft) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::TopLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopRight) {
      Size topLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index topRightIndex(submapIndex(0), 0);
      Size topRightSize(submapBufferSize(0), submapBufferSize(1) - topLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(topRightIndex, topRightSize, BufferRegion::Quadrant::TopRight));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomLeft) {
      Size topLeftSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index bottomLeftIndex(0, submapIndex(1));
      Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomLeftIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {
      Size topLeftSize(bufferSize(0) - submapIndex(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topLeftSize, BufferRegion::Quadrant::TopLeft));

      Index topRightIndex(submapIndex(0), 0);
      Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1) - topLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(topRightIndex, topRightSize, BufferRegion::Quadrant::TopRight));

      Index bottomLeftIndex(0, submapIndex(1));
      Size bottomLeftSize(submapBufferSize(0) - topLeftSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(bottomLeftIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));

      Index bottomRightIndex = Index::Zero();
      Size bottomRightSize(bottomLeftSize(0), topRightSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  } else if (quadrantOfTopLeft == BufferRegion::Quadrant::TopRight) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::TopRight) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::TopRight));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {

      Size topRightSize(bufferSize(0) - submapIndex(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, topRightSize, BufferRegion::Quadrant::TopRight));

      Index bottomRightIndex(0, submapIndex(1));
      Size bottomRightSize(submapBufferSize(0) - topRightSize(0), submapBufferSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  } else if (quadrantOfTopLeft == BufferRegion::Quadrant::BottomLeft) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomLeft) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::BottomLeft));
      return true;
    }

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {
      Size bottomLeftSize(submapBufferSize(0), bufferSize(1) - submapIndex(1));
      submapBufferRegions.push_back(BufferRegion(submapIndex, bottomLeftSize, BufferRegion::Quadrant::BottomLeft));

      Index bottomRightIndex(submapIndex(0), 0);
      Size bottomRightSize(submapBufferSize(0), submapBufferSize(1) - bottomLeftSize(1));
      submapBufferRegions.push_back(BufferRegion(bottomRightIndex, bottomRightSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  } else if (quadrantOfTopLeft == BufferRegion::Quadrant::BottomRight) {

    if (quadrantOfBottomRight == BufferRegion::Quadrant::BottomRight) {
      submapBufferRegions.push_back(BufferRegion(submapIndex, submapBufferSize, BufferRegion::Quadrant::BottomRight));
      return true;
    }

  }

  return false;
}

bool incrementIndex(Index& index, const Size& bufferSize, const Index& bufferStartIndex)
{
  Index unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);

  // Increment index.
  if (unwrappedIndex(1) + 1 < bufferSize(1)) {
    // Same row.
    unwrappedIndex[1]++;
  } else {
    // Next row.
    unwrappedIndex[0]++;
    unwrappedIndex[1] = 0;
  }

  // End of iterations reached.
  if (!checkIfIndexWithinRange(unwrappedIndex, bufferSize)) return false;

  // Return true iterated index.
  index = getBufferIndexFromIndex(unwrappedIndex, bufferSize, bufferStartIndex);
  return true;
}

/**
 * @function    [incrementIndexForSubmap]
 * @description [在子地图迭代的时候，递增索引值]
 * @param       submapIndex              [子地图索引值，初始值为0]
 * @param       index                    [父地图下，子地图的索引中心]
 * @param       submapTopLeftIndex       [和index类似]
 * @param       submapBufferSize         [子地图的大小，一般指cell的个数]
 * @param       bufferSize               [实际地图大小]
 * @param       bufferStartIndex         [实际地图的其实索引值]
 * @return
 */
bool incrementIndexForSubmap(Index& submapIndex, Index& index, const Index& submapTopLeftIndex,
                             const Size& submapBufferSize, const Size& bufferSize,
                             const Index& bufferStartIndex)
{
    // Copy the data first, only copy it back if everything is within range.
    Index tempIndex = index;
    Index tempSubmapIndex = submapIndex;
    // 1. 增加索引值
    // 索引的时候是按照y轴方向增加的
    // Increment submap index.
    if (tempSubmapIndex[1] + 1 < submapBufferSize[1])
    {
        // Same row.
        tempSubmapIndex[1]++;
    }
    else
    {
        // Next row.
        tempSubmapIndex[0]++;
        tempSubmapIndex[1] = 0;
    }

    // 2. 判断增加后的索引值是否还满足要求（是否超过子地图范围）
    // End of iterations reached.
    if (!checkIfIndexWithinRange(tempSubmapIndex, submapBufferSize)) return false;

    // Get corresponding index in map.
    // 3. 计算子地图索引中心距离父地图索引1中心的cell个数
    Index unwrappedSubmapTopLeftIndex = getIndexFromBufferIndex(submapTopLeftIndex, bufferSize, bufferStartIndex);
    // 4. 由3中计算出来的cell个数 + 1中计算出来的cell个数 = 当前迭代后位置句目的图索引中心的cell个数，然后在计算索引值即可
    tempIndex = getBufferIndexFromIndex(unwrappedSubmapTopLeftIndex + tempSubmapIndex, bufferSize, bufferStartIndex);

    // Copy data back.
    index = tempIndex;
    submapIndex = tempSubmapIndex;
    return true;
}

/**
 * @function    [getIndexFromBufferIndex]
 * @description [从循环的buffer中得到展开的buffer的index]
 * @param       bufferIndex      [索引值]
 * @param       bufferSize       [索引区大小]
 * @param       bufferStartIndex [开始索引的索引值]
 * @return                       [description]
 */
Index getIndexFromBufferIndex(const Index& bufferIndex, const Size& bufferSize,
                              const Index& bufferStartIndex)
{
    if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return bufferIndex;
    
    // 当前索引值到最初索引值的cell单元数,注意是index的减法
    Index index = bufferIndex - bufferStartIndex;
    // 对索引值的一些异常做处理
    mapIndexWithinRange(index, bufferSize);
    return index;
}

size_t getLinearIndexFromIndex(const Index& index, const Size& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return index(1) * bufferSize(0) + index(0);
  return index(0) * bufferSize(1) + index(1);
}

Index getIndexFromLinearIndex(const size_t linearIndex, const Size& bufferSize, const bool rowMajor)
{
  if (!rowMajor) return Index((int)linearIndex % bufferSize(0), (int)linearIndex / bufferSize(0));
  return Index((int)linearIndex / bufferSize(1), (int)linearIndex % bufferSize(1));
}

void getIndicesForRegion(const Index& regionIndex, const Size& regionSize,
                         std::vector<Index> indices)
{
//  for (int i = line.index_; col < line.endIndex(); col++) {
//    for (int i = 0; i < getSize()(0); i++) {
//
//    }
//  }
}

void getIndicesForRegions(const std::vector<Index>& regionIndeces, const Size& regionSizes,
                          std::vector<Index> indices)
{
}

bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3i& colorVector)
{
  colorVector(0) = (colorValue >> 16) & 0x0000ff;
  colorVector(1) = (colorValue >> 8) & 0x0000ff;
  colorVector(2) =  colorValue & 0x0000ff;
  return true;
}

bool colorValueToVector(const unsigned long& colorValue, Eigen::Vector3f& colorVector)
{
  Eigen::Vector3i tempColorVector;
  colorValueToVector(colorValue, tempColorVector);
  colorVector = ((tempColorVector.cast<float>()).array() / 255.0).matrix();
  return true;
}

bool colorValueToVector(const float& colorValue, Eigen::Vector3f& colorVector)
{
  const unsigned long tempColorValue = *reinterpret_cast<const unsigned long*>(&colorValue);
  colorValueToVector(tempColorValue, colorVector);
  return true;
}

bool colorVectorToValue(const Eigen::Vector3i& colorVector, unsigned long& colorValue)
{
  colorValue = ((int)colorVector(0)) << 16 | ((int)colorVector(1)) << 8 | ((int)colorVector(2));
  return true;
}

void colorVectorToValue(const Eigen::Vector3i& colorVector, float& colorValue)
{
    int color = (colorVector(0) << 16) + (colorVector(1) << 8) + colorVector(2);
    colorValue = *reinterpret_cast<float*>(&color);
}

}  // namespace

