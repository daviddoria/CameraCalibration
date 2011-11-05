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

Eigen::MatrixXd ComputeNormalizationTransform(const Point2DVector& points)
{
  unsigned int numberOfPoints = points.size();

  Eigen::Vector2d centroid = Centroid(points);

  Point2DVector centeredPoints(numberOfPoints);
  // Shift the origin of the points to the centroid
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    centeredPoints[i] = points[i] - centroid;
    }

  // Compute the average distance
  float totalDistance = 0.0f;
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    totalDistance += centeredPoints[i].norm();
    }

  float averageDistance = totalDistance / static_cast<float>(numberOfPoints);

  float scale = sqrt(2)/averageDistance;

  // Similarity transforms
  Eigen::MatrixXd similarityTransform(3,3);
  similarityTransform(0,0) = scale;
  similarityTransform(0,1) = 0;
  similarityTransform(0,2) = -scale*centroid(0);
  similarityTransform(1,0) = 0;
  similarityTransform(1,1) = scale;
  similarityTransform(1,2) = -scale*centroid(1);
  similarityTransform(2,0) = 0;
  similarityTransform(2,1) = 0;
  similarityTransform(2,2) = 1;

  std::cout << "SimilarityTransform: " << similarityTransform << std::endl;

  return similarityTransform;
  
}

Eigen::MatrixXd ComputeNormalizationTransform(const Point3DVector& points)
{
  unsigned int numberOfPoints = points.size();

  Eigen::Vector3d centroid = Centroid(points);

  Point3DVector centeredPoints(numberOfPoints);
  
  // Shift the origin of the points to the centroid
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    centeredPoints[i] = points[i] - centroid;
    }

  // Normalize the points so that the average distance from the origin is equal to sqrt(2).
  // Compute the average distance
  float totalDistance = 0.0f;
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    totalDistance += centeredPoints[i].norm();
    }

  float averageDistance = totalDistance / static_cast<float>(numberOfPoints);

  float scale = sqrt(3)/averageDistance;

  Eigen::MatrixXd similarityTransform(4,4);
  similarityTransform(0,0) = scale;
  similarityTransform(0,1) = 0;
  similarityTransform(0,2) = 0;
  similarityTransform(0,3) = -scale*centroid(0);
  similarityTransform(1,0) = 0;
  similarityTransform(1,1) = scale;
  similarityTransform(1,2) = 0;
  similarityTransform(1,3) = -scale*centroid(1);
  similarityTransform(2,0) = 0;
  similarityTransform(2,1) = 0;
  similarityTransform(2,2) = scale;
  similarityTransform(2,3) = -scale*centroid(2);
  similarityTransform(3,0) = 0;
  similarityTransform(3,1) = 0;
  similarityTransform(3,2) = 0;
  similarityTransform(3,3) = 1;

  std::cout << "SimilarityTransform: " << similarityTransform << std::endl;
  
  return similarityTransform;
}

Eigen::MatrixXd ComputeP_NormalizedDLT(const Point2DVector& points2D, const Point3DVector& points3D)
{
  unsigned int numberOfPoints = points2D.size();
  if(points3D.size() != numberOfPoints)
    {
    std::cerr << "The number of 2D points (" << points2D.size() << ") must match the number of 3D points (" << points3D.size() << ")!" << std::endl;
    exit(-1);
    }

  Eigen::MatrixXd similarityTransform2D = ComputeNormalizationTransform(points2D);
  Eigen::MatrixXd similarityTransform3D = ComputeNormalizationTransform(points3D);

  std::cout << "Computed similarity transforms." << std::endl;
  
  Point2DVector transformed2DPoints(numberOfPoints);
  Point3DVector transformed3DPoints(numberOfPoints);

  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    Eigen::VectorXd point2Dhomogeneous = points2D[i].homogeneous();
    Eigen::VectorXd point2Dtransformed = similarityTransform2D * point2Dhomogeneous;
    transformed2DPoints[i] = point2Dtransformed.hnormalized();

    Eigen::VectorXd point3Dhomogeneous = points3D[i].homogeneous();
    Eigen::VectorXd point3Dtransformed = similarityTransform3D * point3Dhomogeneous;
    transformed3DPoints[i] = point3Dtransformed.hnormalized();
  
    //transformed2DPoints[i] = (similarityTransform2D * points2D[i].homogeneous()).hnormalized();
    //transformed3DPoints[i] = (similarityTransform3D * points3D[i].homogeneous()).hnormalized();
    }

  std::cout << "Transformed points." << std::endl;
  
  // Compute the Camera Projection Matrix

  Eigen::MatrixXd A(2*numberOfPoints,12);
  for(unsigned int i = 0; i < numberOfPoints; ++i)
    {
    A(2*i, 0) = 0;
    A(2*i, 1) = 0;
    A(2*i, 2) = 0;
    A(2*i, 3) = 0;
    A(2*i, 4) = transformed3DPoints[i](0);
    A(2*i, 5) = transformed3DPoints[i](1);
    A(2*i, 6) = transformed3DPoints[i](2);
    A(2*i, 7) = 1;
    A(2*i, 8) = -transformed2DPoints[i](1) * transformed3DPoints[i](0);
    A(2*i, 9) = -transformed2DPoints[i](1) * transformed3DPoints[i](1);
    A(2*i, 10) = -transformed2DPoints[i](1) * transformed3DPoints[i](2);
    A(2*i, 11) = -transformed2DPoints[i](1);
    
    A(2*i+1, 0) = transformed3DPoints[i](0);
    A(2*i+1, 1) = transformed3DPoints[i](1);
    A(2*i+1, 2) = transformed3DPoints[i](2);
    A(2*i+1, 3) = 1;
    A(2*i+1, 4) = 0;
    A(2*i+1, 5) = 0;
    A(2*i+1, 6) = 0;
    A(2*i+1, 7) = 0;
    A(2*i+1, 8) = -transformed2DPoints[i](0) * transformed3DPoints[i](0);
    A(2*i+1, 9) = -transformed2DPoints[i](0) * transformed3DPoints[i](1);
    A(2*i+1, 10) = -transformed2DPoints[i](0) * transformed3DPoints[i](2);
    A(2*i+1, 11) = -transformed2DPoints[i](0);
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
  P = similarityTransform2D.inverse()*P*similarityTransform3D; // 3x3 * 3x4 * 4x4 = 4x4

  return P;
}
