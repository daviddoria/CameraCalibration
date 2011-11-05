#include "CameraCalibration.h"

#include <iostream>



Eigen::MatrixXd ComputeP_NormalizedDLT(const Point2DVector& points2D, const Point3DVector& points3D)
{
  unsigned int numberOfPoints = points2D.size();
  if(points3D.size() != numberOfPoints)
    {
    std::cerr << "The number of 2D points (" << points2D.size() << ") must match the number of 3D points (" << points3D.size() << ")!" << std::endl;
    exit(-1);
    }

  Eigen::MatrixXd similarityTransform2D = ComputeNormalizationTransform<Eigen::Vector2d>(points2D);
  Eigen::MatrixXd similarityTransform3D = ComputeNormalizationTransform<Eigen::Vector3d>(points3D);

//   std::cout << "Computed similarity transforms:" << std::endl;
//   std::cout << "similarityTransform2D: " << similarityTransform2D << std::endl;
//   std::cout << "similarityTransform3D: " << similarityTransform3D << std::endl;
//   
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
