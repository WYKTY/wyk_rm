#ifndef ANTI_GYRO_H
#define ANTI_GYRO_H

#include"define.h"
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>

#define SOLVEPNP
#define PROJECTION

std::vector<cv::Point2d> solve_unvisual(std::vector<cv::Point2d> armo_2);
bool ClockWise(cv::Point2d a, cv::Point2d b, cv::Point2d cent);

#endif // ANTI_GYRO_H
