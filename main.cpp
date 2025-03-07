/**
 * @file main.cpp
 * @brief Implementation of the QuickHull algorithm to compute the convex hull of a set of 3D points.
 */

#include <iostream>
#include <set>
#include <vector>
#include "glm/geometric.hpp"
#include "glm/vec3.hpp"

/**
 * @brief Computes the signed distance from a point to a plane defined by three vertices.
 *
 * @param point The point whose distance to the plane is to be measured.
 * @param v0 The first vertex of the plane.
 * @param v1 The second vertex of the plane.
 * @param v2 The third vertex of the plane.
 * @return The signed distance from the point to the plane (positive if the point is above the plane, negative otherwise).
 */
float pointToPlaneDistance(const glm::vec3& point, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
    glm::vec3 normal = glm::cross(v1 - v0, v2 - v0);

    // Check to avoid errors due to a degenerate triangle (collinear points)
    if (glm::length(normal) < 1e-6f) return 0.0f;

    normal = glm::normalize(normal);
    return glm::dot(normal, point - v0);
}

/**
 * @brief Recursive function that constructs the convex hull using the QuickHull algorithm.
 *
 * @param points Vector containing all input points.
 * @param hullIndices Set of indices of the points that belong to the convex hull.
 * @param i0 Index of the first point forming an initial triangle.
 * @param i1 Index of the second point forming an initial triangle.
 * @param i2 Index of the third point forming an initial triangle.
 */
void quickHullRecursive(const std::vector<glm::vec3>& points, std::set<int>& hullIndices, const int i0, const int i1, const int i2) {
    float maxDist = 0;
    int farthestIdx = -1;

    // Find the farthest point from the plane formed by i0, i1, and i2
    for (int i = 0; i < points.size(); i++) {
        if (hullIndices.contains(i)) continue;

        const float dist = pointToPlaneDistance(points[i], points[i0], points[i1], points[i2]);
        if (dist > maxDist) {
            maxDist = dist;
            farthestIdx = i;
        }
    }

    // If no point is outside the plane, the triangle is a face of the convex hull
    if (farthestIdx == -1) {
        hullIndices.insert(i0);
        hullIndices.insert(i1);
        hullIndices.insert(i2);
        return;
    }

    // Recursively form new triangles
    quickHullRecursive(points, hullIndices, i0, i1, farthestIdx);
    quickHullRecursive(points, hullIndices, i1, i2, farthestIdx);
    quickHullRecursive(points, hullIndices, i2, i0, farthestIdx);
}

/**
 * @brief Computes the convex hull of a set of 3D points using the QuickHull algorithm.
 *
 * @param points Vector of input points.
 * @return Vector of points that belong to the convex hull.
 */
std::vector<glm::vec3> computeConvexHull(const std::vector<glm::vec3>& points) {
    std::set<int> hullIndices;

    if (points.size() < 4) return points; // At least 4 points are needed to form a 3D volume

    // Find the most extreme points in X, Y, and Z
    int minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;
    for (int i = 1; i < points.size(); i++) {
        if (points[i].x < points[minX].x) minX = i;
        if (points[i].x > points[maxX].x) maxX = i;
        if (points[i].y < points[minY].y) minY = i;
        if (points[i].y > points[maxY].y) maxY = i;
        if (points[i].z < points[minZ].z) minZ = i;
        if (points[i].z > points[maxZ].z) maxZ = i;
    }

    // Initial tetrahedron construction
    quickHullRecursive(points, hullIndices, minX, maxX, minY);
    quickHullRecursive(points, hullIndices, maxX, minY, maxY);
    quickHullRecursive(points, hullIndices, minY, maxY, minX);
    quickHullRecursive(points, hullIndices, maxY, minX, maxX);

    // Create the final result
    std::vector<glm::vec3> convexHullPoints;
    for (int idx : hullIndices) {
        convexHullPoints.push_back(points[idx]);
    }

    return convexHullPoints;
}

/**
 * @brief Checks if a given set of points forms a convex shape.
 *
 * @param points Vector of points to test.
 * @return true if the set is convex, false otherwise.
 */
bool isConvex(const std::vector<glm::vec3>& points) {
    const std::vector<glm::vec3> convexHull = computeConvexHull(points);

    return convexHull.size() == points.size();
}

/**
 * @brief Main program to test the QuickHull algorithm.
 *
 * @return 0 if execution is successful.
 */
int main() {
    // Set of points forming a cube (convex shape)
    const std::vector<glm::vec3> convex_cube = {
        {0, 0, 0}, {1, 0, 0}, {0 , 1, 0}, {0, 0, 1},
        {1, 1, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 0}
    };

    // Set of points forming a concave shape (with an interior point)
    const std::vector<glm::vec3> concave = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
        {1, 1, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 0}, {0.5, 0.5, 0.5}
    };

    // Convexity test
    std::cout << (isConvex(convex_cube) ? "Convex cube: Yes\n" : "Convex cube: No\n") << std::endl;
    std::cout << (isConvex(concave) ? "Concave shape: Yes\n" : "Concave shape: No\n") << std::endl;

    return 0;
}
