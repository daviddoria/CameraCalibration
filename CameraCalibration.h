#ifndef CAMERACLASS_H
#define CAMERACLASS_H

#include <vector>

#include <Eigen/Dense>

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Point3DVector;

// This function computes the Normalized DLT solution of P from corresponding 2D and 3D points.
Eigen::MatrixXd ComputeP_NormalizedDLT(const Point2DVector&, const Point3DVector&);

Eigen::Vector2d Centroid(const Point2DVector& points);
Eigen::Vector3d Centroid(const Point3DVector& points);

Eigen::MatrixXd ComputeNormalizationTransform(const Point2DVector& points);
Eigen::MatrixXd ComputeNormalizationTransform(const Point3DVector& points);

Eigen::MatrixXd HomogeneousMultiply(const Point3DVector& points);

#endif
