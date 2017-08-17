/*
 * Polygon.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_core/Polygon.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace grid_map {

Polygon::Polygon()
    : timestamp_(0)
{
}

Polygon::Polygon(std::vector<Position> vertices)
    : Polygon()
{
  vertices_ = vertices;
}

Polygon::~Polygon() {}

/**
 * @description     [
 *       判断当前position是否在多边形之内
 *                采用了引射线法
 *     http://www.cnblogs.com/luxiaoxun/p/3722358.html
 *     http://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
 *    以测试点A为发射源，统计以A点纵坐标为界限的两侧的与多边形的交点个数，
 *    在实际操作的时候，一般只统计单侧，下面的算法就只统计了穿过左侧的次数
 *    偶数：在多边形外；     奇数：在多边形内
 *
 *      设要判断点C是否在多边形内，vertices_[i]和vertices_[j]用A，B表示
 *      1. 判断以C点出发的射线是否会穿过线段AB
 *      2. 判断点C是否在线段AB的左侧
 *          参考:http://www.cnblogs.com/sixdaycoder/p/4348374.html
 *      3. 满足1,2条件则使穿过次数加一，最后取2的余即可
 * ]
 * @param point
 * @return
 */
bool Polygon::isInside(const Position& point) const
{
    int cross = 0;
    for (int i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++)
    {
        if ( ((vertices_[i].y() > point.y()) != (vertices_[j].y() > point.y()))
               && (point.x() < (vertices_[j].x() - vertices_[i].x()) * (point.y() - vertices_[i].y()) /
                (vertices_[j].y() - vertices_[i].y()) + vertices_[i].x()) )
        {
          cross++;
        }
    }
    return bool(cross % 2);
}

/**
 * @description     [添加多边形定点]
 * @param vertex
 */
void Polygon::addVertex(const Position& vertex)
{
  vertices_.push_back(vertex);
}

const Position& Polygon::getVertex(const size_t index) const
{
  return vertices_.at(index);
}

void Polygon::removeVertices()
{
  vertices_.clear();
}

const Position& Polygon::operator [](const size_t index) const
{
  return getVertex(index);
}

const std::vector<Position>& Polygon::getVertices() const
{
  return vertices_;
}

// 返回顶点个数
const size_t Polygon::nVertices() const
{
  return vertices_.size();
}

const std::string& Polygon::getFrameId() const
{
  return frameId_;
}

/**
 * @description     [设置地图frameID]
 * @param frameId
 */
void Polygon::setFrameId(const std::string& frameId)
{
    frameId_ = frameId;
}

uint64_t Polygon::getTimestamp() const
{
  return timestamp_;
}

void Polygon::setTimestamp(const uint64_t timestamp)
{
  timestamp_ = timestamp;
}

void Polygon::resetTimestamp()
{
  timestamp_ = 0.0;
}

/**
 * @function        [getArea]
 * @description     [计算多边形的面积
 *     可以直接向量的叉积,具体可以参考
 *     http://blog.csdn.net/xxdddail/article/details/48973269
 *     ]
 * @return
 */
const double Polygon::getArea() const
{
    double area = 0.0;
    int j = vertices_.size() - 1;
    for (int i = 0; i < vertices_.size(); i++)
    {
        area += (vertices_.at(j).x() + vertices_.at(i).x())
                * (vertices_.at(j).y() - vertices_.at(i).y());
        j = i;
    }
    return std::abs(area / 2.0);
}

/**
 * @function        [getCentroid]
 * @description     [计算多边形的质心
 *   也可以通过划分三角形,分别求取各三角形的质心,
 *   然后再以面积为权值,加权求取最终的质心
 *   参考：https://en.wikipedia.org/wiki/Centroid中的多边形
 * ]
 * @return
 */
Position Polygon::getCentroid() const
{
    Position centroid = Position::Zero();
    std::vector<Position> vertices = getVertices();
    vertices.push_back(vertices.at(0));
    double area = 0.0;
    for (int i = 0; i < vertices.size() - 1; i++)
    {
        const double a = vertices[i].x() * vertices[i+1].y() - vertices[i+1].x() * vertices[i].y();
        area += a;
        centroid.x() += a * (vertices[i].x() + vertices[i+1].x());
        centroid.y() += a * (vertices[i].y() + vertices[i+1].y());
    }
    area *= 0.5;
    centroid /= (6.0 * area);
    return centroid;
}

/**
 * @function        [convertToInequalityConstraints]
 * @description     [ 将所有点转换为一个紧包含所有点的不等式约束
 *                      来约束多边形的凸包
 *      Ax<=b
 *      参考：http://cn.mathworks.com/matlabcentral/fileexchange/7895-vert2con-vertices-to-constraints
 *           http://blog.csdn.net/bone_ace/article/details/46239187
 *      ]
*  @param V         [p*n, p为顶点个数，n为描述定点的维数]
 * @param A         [m*n,m为方程的个数]
 * @param b         [m*1]
 * @return
 */
bool Polygon::convertToInequalityConstraints(Eigen::MatrixXd& A, Eigen::VectorXd& b) const
{
    // 建立一个V矩阵(n*2)，一个点占一列，两个列元素为该点的横纵坐标
    Eigen::MatrixXd V(nVertices(), 2);
    for (unsigned int i = 0; i < nVertices(); ++i)
        V.row(i) = vertices_[i];

    // Create k, a list of indices from V forming the convex hull.
    // TODO: Assuming counter-clockwise ordered convex polygon.
    // MATLAB: k = convhulln(V); 求去整个凸包的边缘点
    // K矩阵包含多边形的相邻顶点(一个列向量)
    Eigen::MatrixXi k;
    k.resizeLike(V);
    for (unsigned int i = 0; i < V.rows(); ++i)
        k.row(i) << i, (i+1) % V.rows();

    // 建立一个行向量，c为所有点x，y的均值，含有两个元素(V的行的均值)
    Eigen::RowVectorXd c = V.colwise().mean();
    // 所有点的坐标减去他们的坐标均值
    V.rowwise() -= c;

    // 构建一个n*2的A矩阵，大小与V一致
    A = Eigen::MatrixXd::Constant(k.rows(), V.cols(), NAN);
    unsigned int rc = 0;
    for (unsigned int ix = 0; ix < k.rows(); ++ix)
    {
        Eigen::MatrixXd F(2, V.cols());
        F.row(0) << V.row(k(ix, 0));
        F.row(1) << V.row(k(ix, 1));
        // 对F矩阵做对角分解
        Eigen::FullPivLU<Eigen::MatrixXd> luDecomp(F);
        // 分解前后都满秩
        if (luDecomp.rank() == F.rows())
        {
            // 求解方程 Fan^=I，
            //  | X1    Y1 || a1| |1|
            //  | X2    Y2 || a2|=|1|
            A.row(rc) = F.colPivHouseholderQr().solve(Eigen::VectorXd::Ones(F.rows()));
            ++rc;
        }
    }
    // 取A矩阵的前rc行
    // A=[a1,a2,a3,a4,,,an]^
    // v=[v1,v2,v3,v4,,,vn]^
    // an(vn-c)^=1 ===>  anvn^=anc^+1
    // b = 1 + Ac^
    A = A.topRows(rc);
    b = Eigen::VectorXd::Ones(A.rows());
    b = b + A * c.transpose();

    return true;
}

/**
 * @function        [offsetInward]
 * @swacrption      [
 *      给多边形整体一个向内的偏移,即将所有顶点向内做一个等值的偏移，原多边形的形状不变。
 *      参考：https://en.wikipedia.org/wiki/Straight_skeleton
 *      算法详细介绍：http://blog.csdn.net/happy__888/article/details/315762
 * ]
 * @param margin    [s缩放的距离]
 * @return
 */
bool Polygon::offsetInward(const double margin)
{
    // Create a list of indices of the neighbours of each vertex.
    // TODO: Assuming counter-clockwise ordered convex polygon.
    // 1. 建立一个相邻点的向量，如有三个点1,2,3则，neighbourIndeices为
    //      | 2 1 |
    //      | 0 2 |
    //      | 1 0 |
    std::vector<Eigen::Array2i> neighbourIndices;
    const unsigned int n = nVertices();
    neighbourIndices.resize(n);
    for (unsigned int i = 0; i < n; ++i)
    {
        neighbourIndices[i] << (i > 0 ? (i-1)%n : n-1), (i + 1) % n;
    }
    // 2. 生成新的缩放后的顶点
    std::vector<Position> copy(vertices_);
    for (unsigned int i = 0; i < neighbourIndices.size(); ++i)
    {

        // 2.1 提取由该点出发的两条边
        Eigen::Vector2d v1 = vertices_[neighbourIndices[i](0)] - vertices_[i];
        Eigen::Vector2d v2 = vertices_[neighbourIndices[i](1)] - vertices_[i];
        v1.normalize();
        v2.normalize();
        // 2.2 求取V1和V2的夹角
        const double angle = acos(v1.dot(v2));
        // 2.3 沿合成向量做平移，等于将顶点平移
        copy[i] += margin / sin(angle) * (v1 + v2);
    }
    vertices_ = copy;
    return true;
}

/**
 * @function        [fromCircle]
 * @descrption      [
 *          做多边形的逼近圆
 *      如果是这样算法的话，那么半径radius的选择岂不是很重要？
 * ]
 * @param center
 * @param radius
 * @param nVertices
 * @return
 */
Polygon Polygon::fromCircle(const Position center, const double radius,
                                  const int nVertices)
{
    Eigen::Vector2d centerToVertex(radius, 0.0), centerToVertexTemp;
    Polygon polygon;
    for (int j = 0; j < nVertices; j++)
    {
        // 1. 计算旋转角 theta = (2*j*π) / (n-1)(将圆弧平分)
        double theta = j * 2 * M_PI / (nVertices - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        // 2. 计算旋转theta后的centerToVertex(顺时针旋转)
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        // 最终的圆的顶点还有加上圆心
        polygon.addVertex(center + centerToVertexTemp);
    }
    return polygon;
}

/**
 * @function        [convexHullOfTwoCircles]
 * @description     [
 *          做带有一个凸包的两个圆，并将其最终生成的多边形
 *      和fromCircle函数类似，做了两个圆来近似原多边形的凸包
 *      但是，像这样的圆的半径是怎么确定的，如果半径过小，怎么表示凸包呢？
 * ]
 * @param center1
 * @param center2
 * @param radius
 * @param nVertices
 * @return
 */
Polygon Polygon::convexHullOfTwoCircles(const Position center1,
                                   const Position center2, const double radius,
                                   const int nVertices)
{
    // 1. 确定centerToVertex的大小和位置
    Eigen::Vector2d centerToVertex, centerToVertexTemp;
    centerToVertex = center2 - center1;
    centerToVertex.normalize();
    centerToVertex *= radius;

    // 2.做以center1为圆心的近似圆
    grid_map::Polygon polygon;
    for (int j = 0; j < ceil(nVertices / 2.0); j++)
    {
        // 2.1 计算旋转角度 theta = π/2 + j* π/(0.5*N-1)
        double theta = M_PI_2 + j * M_PI / (ceil(nVertices / 2.0) - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        polygon.addVertex(center1 + centerToVertexTemp);
    }
    // 3.做以center2为圆心的近似圆
    for (int j = 0; j < ceil(nVertices / 2.0); j++)
    {
        double theta = 3 * M_PI_2 + j * M_PI / (ceil(nVertices / 2.0) - 1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix() * centerToVertex;
        polygon.addVertex(center2 + centerToVertexTemp);
    }
    return polygon;
}

/**
 * @function        [convexHull]
 * @description     [
 *          计算两个多边形的联合凸包
 *      参考：http://www.iro.umontreal.ca/~plante/compGeom/algorithm.html
 *          ]
 * @param polygon1
 * @param polygon2
 * @return
 */
Polygon Polygon::convexHull(Polygon& polygon1, Polygon& polygon2)
{
    // 1. 写入所有顶点
    std::vector<Position> vertices;
    // 1.1 确定vertices的大小
    vertices.reserve(polygon1.nVertices() + polygon2.nVertices());
    // 1.2 vertices的前半部分放入多边形1的所有顶点
    vertices.insert(vertices.end(), polygon1.getVertices().begin(), polygon1.getVertices().end());
    // 1.3 vertices的后半部分放入多边形2的所有顶点
    vertices.insert(vertices.end(), polygon2.getVertices().begin(), polygon2.getVertices().end());

    std::vector<Position> hull(vertices.size()+1);

    // Sort points lexicographically
    // 对vertices中的点按照自左下到右上的顺序排列
    std::sort(vertices.begin(), vertices.end(), sortVertices);

    int k = 0;
    // Build lower hull
    // 建立凸包的下边缘
    for (int i = 0; i < vertices.size(); ++i)
    {
        // 点（k-1）在点i的左侧(上侧)，则这个点不要
        while (k >= 2
            && computeCrossProduct2D(hull.at(k - 1) - hull.at(k - 2),
                                     vertices.at(i) - hull.at(k - 2)) <= 0)
          k--;
        hull.at(k++) = vertices.at(i);
    }

    // Build upper hull
    // 建立上边缘，倒序
    for (int i = vertices.size() - 2, t = k + 1; i >= 0; i--)
    {
        //
        while (k >= t
            && computeCrossProduct2D(hull.at(k - 1) - hull.at(k - 2),
                                     vertices.at(i) - hull.at(k - 2)) <= 0)
          k--;
        hull.at(k++) = vertices.at(i);
    }
    hull.resize(k - 1);

    Polygon polygon(hull);
    return polygon;
}

/**
 * @description     [判断两个向量是否按照字典排序]
 * @param vector1
 * @param vector2
 * @return
 */
bool Polygon::sortVertices(const Eigen::Vector2d& vector1,
                           const Eigen::Vector2d& vector2)
{
    // 当点1位于点2的左侧或者下侧时返回真值
    return (vector1.x() < vector2.x()
        || (vector1.x() == vector2.x() && vector1.y() < vector2.y()));
}

/**
 * @description     [计算两个向量的叉乘
 *      V1×V2 = x1*y2 - y1*x2
 *      点V1在点V2的右侧，叉乘大于0，垂直等于0，左侧小于0
 *      ]
 *
 * @param vector1
 * @param vector2
 * @return
 */
double Polygon::computeCrossProduct2D(const Eigen::Vector2d& vector1,
                                      const Eigen::Vector2d& vector2)
{
    return (vector1.x() * vector2.y() - vector1.y() * vector2.x());
}

} /* namespace grid_map */

