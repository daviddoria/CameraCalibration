#ifndef CAMERACLASS_H
#define CAMERACLASS_H

#include <vector>

#include <Eigen/Dense>

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Point3DVector;

// This function computes the Normalized DLT solution of P from corresponding 2D and 3D points.
Eigen::MatrixXd ComputeP_NormalizedDLT(const Point2DVector&, const Point3DVector&);

template<typename T>
T Centroid(const typename std::vector<T,typename Eigen::aligned_allocator<T> >& points);

template<typename T>
Eigen::MatrixXd ComputeNormalizationTransform(const typename std::vector<T,typename Eigen::aligned_allocator<T> >& points);

Eigen::MatrixXd HomogeneousMultiply(const Point3DVector& points);

Eigen::MatrixXd Reshape(const Eigen::VectorXd& vec, const unsigned int rows, const unsigned int cols);

#include "CameraCalibration.hxx"

#endif
