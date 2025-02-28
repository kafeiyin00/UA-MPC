#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <omp.h> // Include OpenMP header
#include <opencv2/opencv.hpp> // Include OpenCV header


// Constants
const float PI = 3.14159265359f;

// Function to convert from Cartesian to spherical coordinates
void cartesianToSpherical(float x, float y, float z, float& theta, float& phi, float& r) {
    r = std::sqrt(x * x + y * y + z * z);
    theta = std::atan2(y, x); // Azimuth angle in xy-plane from x-axis
    phi = std::acos(z / r);   // Polar angle from z-axis
}

// Function to generate a panoramic depth map using OpenMP for acceleration and OpenCV for depth map representation
void generatePanoramicDepthMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int width, int height,
                               cv::Mat& depthMap,
                               cv::Mat& idxMap,
                               const Eigen::Vector3f& offset,
                               const Eigen::Quaternionf& q) {
    // Initialize depth map and index map
    depthMap = cv::Mat(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));
    idxMap = cv::Mat(height, width, CV_32SC1, cv::Scalar(-1)); // Initialize index map with -1

    // OpenMP settings
    int numThreads = omp_get_max_threads();

    // Create a depth map and index map for each thread to avoid data races
    std::vector<cv::Mat> depthMapsPerThread(numThreads, cv::Mat(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity())));
    std::vector<cv::Mat> idxMapsPerThread(numThreads, cv::Mat(height, width, CV_32SC1, cv::Scalar(-1)));

    // Parallel processing of point cloud
    #pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        cv::Mat& localDepthMap = depthMapsPerThread[threadId];
        cv::Mat& localIdxMap = idxMapsPerThread[threadId];

        // Each thread processes a portion of the point cloud
        #pragma omp for nowait
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            // Transform the point using the given offset and rotation
            Eigen::Vector3f pt = q.inverse() * (Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z) - offset);

            float x = pt(0);
            float y = pt(1);
            float z = pt(2);

            // Convert to spherical coordinates
            float theta, phi, r;
            cartesianToSpherical(x, y, z, theta, phi, r);

            // Normalize theta from (-pi, pi] to [0, 1]
            float u = (theta + PI) / (2.0f * PI);

            // Normalize phi from [0, pi] to [0, 1]
            float v = phi / PI;

            // Convert to pixel coordinates
            int u_pixel = static_cast<int>(u * (width - 1));
            int v_pixel = static_cast<int>(v * (height - 1));

            // Ensure pixel indices are within bounds
            u_pixel = std::min(std::max(u_pixel, 0), width - 1);
            v_pixel = std::min(std::max(v_pixel, 0), height - 1);

            // Update the local depth map and index map with the minimum depth
            float& depthValue = localDepthMap.at<float>(v_pixel, u_pixel);
            int& idxValue = localIdxMap.at<int>(v_pixel, u_pixel);

            if (r < depthValue) {
                depthValue = r;
                idxValue = static_cast<int>(i); // Cast to int to match the CV_32SC1 type
            }
        }
    }

    // Merge local depth maps and index maps into the final depth map and index map
    for (int threadId = 0; threadId < numThreads; ++threadId) {
        cv::Mat& localDepthMap = depthMapsPerThread[threadId];
        cv::Mat& localIdxMap = idxMapsPerThread[threadId];

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                float localDepth = localDepthMap.at<float>(v, u);
                int localIdx = localIdxMap.at<int>(v, u);
                float& globalDepth = depthMap.at<float>(v, u);
                int& globalIdx = idxMap.at<int>(v, u);

                if (localDepth < globalDepth) {
                    globalDepth = localDepth;
                    globalIdx = localIdx;
                }
            }
        }
    }
}
