#include <iostream>

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>

#include "CameraCalibration.h"

std::vector<vgl_point_2d<double> > Create2DPoints();
std::vector<vgl_point_3d<double> > Create3DPoints();

int main(int argc, char *argv[])
{

  std::vector<vgl_point_2d<double> > points2D = Create2DPoints();
  std::vector<vgl_point_3d<double> > points3D = Create3DPoints();

  return 0;
}


std::vector<vgl_point_2d<double> > Create2DPoints()
{
  std::vector<vgl_point_2d<double> > points;

  points.push_back(vgl_point_2d<double> (950, 1058));
  points.push_back(vgl_point_2d<double> (964, 1279));
  points.push_back(vgl_point_2d<double> (1163, 1259));
  points.push_back(vgl_point_2d<double> (1212, 1045));
  points.push_back(vgl_point_2d<double> (1221, 1265));
  points.push_back(vgl_point_2d<double> (1048, 1286));
  points.push_back(vgl_point_2d<double> (1134, 1491));
  points.push_back(vgl_point_2d<double> (1057, 1488));
  points.push_back(vgl_point_2d<double> (1039, 1007));
  points.push_back(vgl_point_2d<double> (1446, 1013));

  return points;
  //std::cout << "a.x: " << a.x() << std::endl;
}

std::vector<vgl_point_3d<double> > Create3DPoints()
{
  std::vector<vgl_point_3d<double> > points;

  points.push_back(vgl_point_3d<double> (10.2891, -1.16362, 1.30919));
  points.push_back(vgl_point_3d<double> (10.2794, -1.25148, 0.550674));
  points.push_back(vgl_point_3d<double> (9.80501, -1.85298, 0.623428));
  points.push_back(vgl_point_3d<double> (10.1614, -2.05843, 1.40193));
  points.push_back(vgl_point_3d<double> (10.1373, -2.12215, 0.637558));
  points.push_back(vgl_point_3d<double> (10.2417, -1.54201, 0.535385));
  points.push_back(vgl_point_3d<double> (10.0461, -1.83568, -0.148849));
  points.push_back(vgl_point_3d<double> (10.2388, -1.60673, -0.150742));
  points.push_back(vgl_point_3d<double> (17.0905, -2.43593, 2.53593));
  points.push_back(vgl_point_3d<double> (16.953, -4.86699, 2.65682));

  return points;
}
