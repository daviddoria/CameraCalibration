#include <iostream>

#include <Eigen/Dense>

#include "CameraCalibration.h"

CameraCalibration::Point2DVector Create2DPoints();
CameraCalibration::Point3DVector Create3DPoints();

int main(int argc, char *argv[])
{

  CameraCalibration::Point2DVector points2D = Create2DPoints();
  CameraCalibration::Point3DVector points3D = Create3DPoints();

  Eigen::MatrixXd P = CameraCalibration::ComputeP_NormalizedDLT(points2D, points3D);

  std::cout << "P: " << P << std::endl;

  /* Matlab says:
   1.3259e+01  -7.1752e+01   5.9339e+00  -5.8879e+00
   3.1995e+01  -9.5948e+00  -6.4394e+01  -8.9752e+00
   2.2462e-02  -4.7089e-03   2.3507e-03  -6.1825e-03
 */

  /* this program says:
P:    -13.2592     71.7521    -5.93392     5.88788
   -31.9947     9.59482     64.3943     8.97516
 -0.0224625  0.00470891 -0.00235072  0.00618254
 */

  return 0;
}


CameraCalibration::Point2DVector Create2DPoints()
{
  CameraCalibration::Point2DVector points;
  
  points.push_back(Eigen::Vector2d (950, 1058));
  points.push_back(Eigen::Vector2d (964, 1279));
  points.push_back(Eigen::Vector2d (1163, 1259));
  points.push_back(Eigen::Vector2d (1212, 1045));
  points.push_back(Eigen::Vector2d (1221, 1265));
  points.push_back(Eigen::Vector2d (1048, 1286));
  points.push_back(Eigen::Vector2d (1134, 1491));
  points.push_back(Eigen::Vector2d (1057, 1488));
  points.push_back(Eigen::Vector2d (1039, 1007));
  points.push_back(Eigen::Vector2d (1446, 1013));

  return points;
  //std::cout << "a.x: " << a.x() << std::endl;
}

CameraCalibration::Point3DVector Create3DPoints()
{
  CameraCalibration::Point3DVector points;

  points.push_back(Eigen::Vector3d (10.2891, -1.16362, 1.30919));
  points.push_back(Eigen::Vector3d (10.2794, -1.25148, 0.550674));
  points.push_back(Eigen::Vector3d (9.80501, -1.85298, 0.623428));
  points.push_back(Eigen::Vector3d (10.1614, -2.05843, 1.40193));
  points.push_back(Eigen::Vector3d (10.1373, -2.12215, 0.637558));
  points.push_back(Eigen::Vector3d (10.2417, -1.54201, 0.535385));
  points.push_back(Eigen::Vector3d (10.0461, -1.83568, -0.148849));
  points.push_back(Eigen::Vector3d (10.2388, -1.60673, -0.150742));
  points.push_back(Eigen::Vector3d (17.0905, -2.43593, 2.53593));
  points.push_back(Eigen::Vector3d (16.953, -4.86699, 2.65682));

  return points;
}
