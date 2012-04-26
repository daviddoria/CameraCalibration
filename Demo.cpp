#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "CameraCalibration.h"

int main(int argc, char *argv[])
{
  std::string points2DfileName = argv[1];
  std::string points3DfileName = argv[2];
  
  Point2DVector points2D = LoadPoints2D(points2DfileName);
  Point3DVector points3D = LoadPoints3D(points3DfileName);

  Eigen::MatrixXd P = ComputeP_NormalizedDLT(points2D, points3D);

  std::cout << "P: " << P << std::endl;

  std::string outputFileName = "P.txt";
  std::ofstream fout(outputFileName.c_str());

  fout << P << std::endl;

  fout.close();
  return 0;
}
