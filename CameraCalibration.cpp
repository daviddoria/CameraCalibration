#include "CameraCalibration.h"

#include <iostream>

Eigen::Vector2d Centroid(const Point2DVector& points)
{
  Eigen::Vector2d centroid(0.0f, 0.0f);

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    centroid += points[i];
    }

  float numberOfPoints = static_cast<float>(points.size());
  centroid /= numberOfPoints;
  
  return centroid;
}

Eigen::Vector3d Centroid(const Point3DVector& points)
{
  Eigen::Vector3d centroid(0.0f, 0.0f, 0.0f);

  for(unsigned int i = 0; i < points.size(); ++i)
    {
    centroid += points[i];
    }

  float numberOfPoints = static_cast<float>(points.size());
  centroid /= numberOfPoints;

  return centroid;
}

Eigen::MatrixXd ComputeP(const Point2DVector& points2Dinput, const Point3DVector& points3Dinput)
{
  Point2DVector points2D = points2Dinput;
  Point3DVector points3D = points3Dinput;
  
  unsigned int numberOfPoints = points2D.size();
  if(points3D.size() != numberOfPoints)
    {
    std::cerr << "The number of 2D points (" << points2D.size() << ") must match the number of 3D points (" << points3D.size() << ")!" << std::endl;
    exit(-1);
    }

  Eigen::Vector2d centroid2D = Centroid(points2D);
  Eigen::Vector3d centroid3D = Centroid(points3D);

  std::cout << "Centroid2D: " << centroid2D << std::endl;
  std::cout << "Centroid3D: " << centroid3D << std::endl;

  // Shift the origin of the points to the centroid
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    points2D[i] -= centroid2D;
    points3D[i] -= centroid3D;
    }

  // Normalize the points so that the average distance from the origin is equal to sqrt(2).
  // Compute the average distance
  float totalDistance2D = 0.0f;
  float totalDistance3D = 0.0f;
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    totalDistance2D += points2D[i].norm();
    totalDistance3D += points3D[i].norm();
    }

  float averageDistance2D = totalDistance2D / static_cast<float>(numberOfPoints);
  float averageDistance3D = totalDistance3D / static_cast<float>(numberOfPoints);

  float scale2D = sqrt(2)/averageDistance2D;
  float scale3D = sqrt(3)/averageDistance3D;

  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    points2D[i] *= scale2D;
    points3D[i] *= scale3D;
    }

  std::cout << "Normalized 2D point 0: " << points2D[0] << std::endl;
  std::cout << "Normalized 3D point 0: " << points3D[0] << std::endl;

  // Similarity transforms
  Eigen::MatrixXd SimilarityTransform2D(3,3);
  SimilarityTransform2D(0,0) = scale2D;
  SimilarityTransform2D(0,1) = 0;
  SimilarityTransform2D(0,2) = -scale2D*centroid2D(0);
  SimilarityTransform2D(1,0) = 0;
  SimilarityTransform2D(1,1) = scale2D;
  SimilarityTransform2D(1,2) = -scale2D*centroid2D(1);
  SimilarityTransform2D(2,0) = 0;
  SimilarityTransform2D(2,1) = 0;
  SimilarityTransform2D(2,2) = 1;

  Eigen::MatrixXd SimilarityTransform3D(4,4);
  SimilarityTransform3D(0,0) = scale3D;
  SimilarityTransform3D(0,1) = 0;
  SimilarityTransform3D(0,2) = 0;
  SimilarityTransform3D(0,3) = -scale3D*centroid3D(0);
  SimilarityTransform3D(1,0) = 0;
  SimilarityTransform3D(1,1) = scale3D;
  SimilarityTransform3D(1,2) = 0;
  SimilarityTransform3D(1,3) = -scale3D*centroid3D(1);
  SimilarityTransform3D(2,0) = 0;
  SimilarityTransform3D(2,1) = 0;
  SimilarityTransform3D(2,2) = scale3D;
  SimilarityTransform3D(2,3) = -scale3D*centroid3D(2);
  SimilarityTransform3D(3,0) = 0;
  SimilarityTransform3D(3,1) = 0;
  SimilarityTransform3D(3,2) = 0;
  SimilarityTransform3D(3,3) = 1;

  std::cout << "SimilarityTransform2D: " << SimilarityTransform2D << std::endl;
  std::cout << "SimilarityTransform3D: " << SimilarityTransform3D << std::endl;
  
  
  // We cannot use the SimilarityTransform matrices directly to do the transformations that were done manually above because the scale is computed half way through the transformation.
  
  // Compute the Camera Projection Matrix

  Eigen::MatrixXd A(2*numberOfPoints,12);
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    A(2*i, 0) = 0;
    A(2*i, 1) = 0;
    A(2*i, 2) = 0;
    A(2*i, 3) = 0;
    A(2*i, 4) = points3D[i](0);
    A(2*i, 5) = points3D[i](1);
    A(2*i, 6) = points3D[i](2);
    A(2*i, 7) = 1;
    A(2*i, 8) = -points2D[i](1) * points3D[i](0);
    A(2*i, 9) = -points2D[i](1) * points3D[i](1);
    A(2*i, 10) = -points2D[i](1) * points3D[i](2);
    A(2*i, 11) = -points2D[i](1);
    
    A(2*i+1, 0) = points3D[i](0);
    A(2*i+1, 1) = points3D[i](1);
    A(2*i+1, 2) = points3D[i](2);
    A(2*i+1, 3) = 1;
    A(2*i+1, 4) = 0;
    A(2*i+1, 5) = 0;
    A(2*i+1, 6) = 0;
    A(2*i+1, 7) = 0;
    A(2*i+1, 8) = -points2D[i](0) * points3D[i](0);
    A(2*i+1, 9) = -points2D[i](0) * points3D[i](1);
    A(2*i+1, 10) = -points2D[i](0) * points3D[i](2);
    A(2*i+1, 11) = -points2D[i](0);
    }

  std::cout << "A: " << A << std::endl;
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::MatrixXd V = svd.matrixV();
  Eigen::MatrixXd lastColumnOfV = V.col(11);

  Eigen::MatrixXd P(3,4);
  for(unsigned int row = 0; row < 3; ++row)
    {
    for(unsigned int col = 0; col < 4; ++col)
      {
      P(row, col) = lastColumnOfV(row*4 + col);
      }
    }
    
  // Denormalization
  P = SimilarityTransform2D.inverse()*P*SimilarityTransform3D; // 3x3 * 3x4 * 4x4 = 4x4

  return P;
}
