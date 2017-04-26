#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <grid_map_core/grid_map_core.hpp>
#include <chrono>
#include <time.h>

using namespace grid_map;

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    // Create grid map.
    GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
    map.setFrameId("map");
    map.setGeometry(Length(1.2, 2.0), 0.03, Position(0.0, -0.1));
    printf("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1),
    map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

//    time_t tt = time(NULL);
//    tm* time= localtime(&tt);
    // Work with grid map in a loop.
    while (1)
    {
        // Add elevation and surface normal (iterating through grid map and adding data).
        for (GridMapIterator it(map); !it.isPastEnd(); ++it)
        {
            Position position;
            map.getPosition(*it, position);
            map.at("elevation", *it) = -0.04 + 0.2 * std::sin(5.0 * position.y()) * position.x();
            Eigen::Vector3d normal(-0.2 * std::sin( 5.0 * position.y()),
                             -position.x() * std::cos( 5.0 * position.y()), 1.0);
            normal.normalize();
            map.at("normal_x", *it) = normal.x();
            map.at("normal_y", *it) = normal.y();
            map.at("normal_z", *it) = normal.z();
        }

        // Add noise (using Eigen operators).
        map.add("noise", 0.015 * Matrix::Random(map.getSize()(0), map.getSize()(1)));
        map.add("elevation_noisy", map.get("elevation") + map["noise"]);

        // 图层添加异常值
        // Adding outliers (accessing cell by position).
        for (unsigned int i = 0; i < 500; ++i)
        {
            // 生成随机矩阵
            Position randomPosition = Position::Random();

            if (map.isInside(randomPosition))
            {
                map.atPosition("elevation_noisy", randomPosition) = std::numeric_limits<float>::infinity();
          }
        }

        // Filter values for submap (iterators).
        map.add("elevation_filtered", map.get("elevation_noisy"));
        Position topLeftCorner(1.0, 0.4);
        limitPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
        Index startIndex;
        map.getIndex(topLeftCorner, startIndex);
        printf("Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
                 topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

        Size size = (Length(1.2, 0.8) / map.getResolution()).cast<int>();
        SubmapIterator it(map, startIndex, size);
        for (; !it.isPastEnd(); ++it)
        {
            Position currentPosition;
            map.getPosition(*it, currentPosition);
            double radius = 0.1;
            double mean = 0.0;
            double sumOfWeights = 0.0;

            // Compute weighted mean.
            for (CircleIterator circleIt(map, currentPosition, radius); !circleIt.isPastEnd(); ++circleIt)
            {
                if (!map.isValid(*circleIt, "elevation_noisy")) continue;
                Position currentPositionInCircle;
                map.getPosition(*circleIt, currentPositionInCircle);

                // Computed weighted mean based on Euclidian distance.
                double distance = (currentPosition - currentPositionInCircle).norm();
                double weight = pow(radius - distance, 2);
                mean += weight * map.at("elevation_noisy", *circleIt);
                sumOfWeights += weight;
            }

            map.at("elevation_filtered", *it) = mean / sumOfWeights;
          }

        // Show absolute difference and compute mean squared error.
        map.add("error", (map.get("elevation_filtered") - map.get("elevation")).cwiseAbs());
        unsigned int nCells = map.getSize().prod();
        double rootMeanSquaredError = sqrt((map["error"].array().pow(2).sum()) / nCells);
    }

    return 0;
}