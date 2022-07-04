#include"define.h"
#include"anti_gyro.h"
#include"detecte.h"

using namespace std;
using namespace cv;

Mat rvec;
Mat tvec;
Mat rotated_vector;

//内参矩阵
double cam_mar[9] = {
    2178.1,0,1092.4,
    0,2164.6,557.5319,
    0,0,1.00 };

Mat camera_martrix = Mat(3, 3, CV_64FC1, cam_mar);

//畸变系数
double cam_dist[5] = { -0.2207,0.2196,0,0,0 };

Mat camera_distcoeff = Mat(5, 1, CV_64FC1, cam_dist);


std::vector<cv::Point2d> solve_unvisual(std::vector<cv::Point2d> armo_2)
{
    vector<Point2d> armo_4;
    double X_1 = 0, Y_1 = 0;

    for (int i = 0; i < 4; i++)
    {
        X_1 += armo_2[i].x;
        Y_1 += armo_2[i].y;
    }

    Point2d wei_center_1( X_1/4, Y_1 /4);

    double X_2 = 0, Y_2 = 0;

    for (int i = 4; i < 8; i++)
    {
        X_2 += armo_2[i].x;
        Y_2 += armo_2[i].y;
    }

    Point2d wei_center_2( X_2/4, Y_2 /4);

//    for(int i=0;i<4;i++)
//    {
//        for(int j=0;j<3-i;j++)
//        {
//            if(!ClockWise(armo_2[j],armo_2[j+1],wei_center_1))
//            {
//                Point2d temp = armo_2[j];
//                armo_2[j]=armo_2[j+1];
//                armo_2[j+1]=temp;
//            }
//        }
//    }

//    for(int i=4;i<8;i++)
//    {
//        for(int j=4;j<11-i;j++)
//        {
//            if(!ClockWise(armo_2[j],armo_2[j+1],wei_center_1))
//            {
//                Point2d temp = armo_2[j];
//                armo_2[j]=armo_2[j+1];
//                armo_2[j+1]=temp;
//            }
//        }
//    }

    Point2d arm1[4];
    Point2d arm2[4];

    for(int i=0;i<4;i++)
    {
        if(armo_2[i].x<wei_center_1.x&&armo_2[i].y<wei_center_1.y)
            arm1[0]=armo_2[i];
        if(armo_2[i].x>wei_center_1.x&&armo_2[i].y<wei_center_1.y)
            arm1[1]=armo_2[i];
        if(armo_2[i].x<wei_center_1.x&&armo_2[i].y>wei_center_1.y)
            arm1[2]=armo_2[i];
        if(armo_2[i].x>wei_center_1.x&&armo_2[i].y>wei_center_1.y)
            arm1[3]=armo_2[i];
    }

    for(int i=0;i<4;i++)
    {
        if(armo_2[i+4].x<wei_center_2.x&&armo_2[i+4].y<wei_center_2.y)
            arm2[0]=armo_2[i+4];
        if(armo_2[i+4].x>wei_center_2.x&&armo_2[i+4].y<wei_center_2.y)
            arm2[1]=armo_2[i+4];
        if(armo_2[i+4].x<wei_center_2.x&&armo_2[i+4].y>wei_center_2.y)
            arm2[2]=armo_2[i+4];
        if(armo_2[i+4].x>wei_center_2.x&&armo_2[i+4].y>wei_center_2.y)
            arm2[3]=armo_2[i+4];
    }

    if(wei_center_1.x<wei_center_2.x)
    {

        for(int i=0;i<4;i++)
        {
            armo_4.push_back(arm1[i]);
        }

        for(int i=0;i<4;i++)
        {
            armo_4.push_back(arm2[i]);
        }

    }
    else
    {
        for(int i=0;i<4;i++)
        {
            armo_4.push_back(arm2[i]);
        }

        for(int i=0;i<4;i++)
        {
            armo_4.push_back(arm1[i]);
        }
    }




    //二维点集  armo_4
    vector<Point2d> Points2D;
    Points2D.push_back(Point2d(53.2416,942.666));
    Points2D.push_back(Point2d(110.151,0.499058));
    Points2D.push_back(Point2d(1248.63,69.2611));
    Points2D.push_back(Point2d(1191.72,1011.43));


    //三维点集  armor 1
    vector<Point3d> Points3D;
    Points3D.push_back(Point3d(0,0,0));
    Points3D.push_back(Point3d(0, 0, 0));
    Points3D.push_back(Point3d(0,100,0));
    Points3D.push_back(Point3d(80,100,0));


    //三维点集  projectpoint  armor_4
    vector<Point3d> armo_3;


    //三维点集  重投影  armo_4
    vector<Point2d> armo_show;



    //pnp求解
    //solvePnP(Points3D, Points2D, camera_martrix, camera_distcoeff, rvec, tvec, false, CV_EPNP);

    //cout << rvec<<"\n\n"<<tvec<<"\n\n";


#ifdef SOLVEPNP
    //格式转换
//	Rodrigues(rvec, rotated_vector);
//	Eigen::Matrix3d R_n;
//	Eigen::Vector3d T_n;
//	cv2eigen(rotated_vector, R_n);
//	cv2eigen(tvec, T_n);
//	Eigen::Vector3d P_fin;
//	P_fin = -R_n.inverse() * T_n;
//	double distan;
//	distan = sqrt(powf(P_fin[0], 2) + powf(P_fin[1], 2) + powf(P_fin[2], 2));
//	cout << P_fin << endl;
//	cout << P_fin[0] << "  " << P_fin[1] << "  " << P_fin[2] << endl;
//	cout << distan;
#endif //解算

#ifdef PROJECTION

//	projectPoints(points_warn, rvec, tvec, camera_martrix, camera_distcoeff, points_warn_projection);

//	for (int i = 0; i < points_warn_projection.size(); i++)
//	{
//		cout << points_warn_projection[i]<<"\t\t"<<Points2D[i]<<"\n\n";
//	}

#endif //重投影



    return armo_4;
}


//double X = 0, Y = 0;

//	for (int i = 0; i < points_warn_projection.size(); i++)
//	{
//		X += points_warn_projection[i].x;
//		Y += points_warn_projection[i].y;
//	}
//	Point2d wei_center(X/ points_warn_projection.size(),Y/ points_warn_projection.size());
//	for (int i = 0; i < points_warn_projection.size() - 1; i++)
//	{
//		for (int j = 0; j < points_warn_projection.size() - i - 1; j++)
//		{
//			if (ClockWise(points_warn_projection[j], points_warn_projection[j+1],wei_center))
//			{
//				Point2d temp = points_warn_projection[j];
//				points_warn_projection[j] = points_warn_projection[j + 1];
//				points_warn_projection[j + 1] = temp;
//			}
//		}
//	}


bool ClockWise(Point2d a, Point2d b, Point2d cent)
{
    double fork = ((a.x - cent.x) * (b.y - cent.y) - (b.x - cent.x) * (a.y - cent.y));
  //int det = ((a.x - cent.x) * (b.y - cent.y) - (b.x - cent.x) * (a.y - cent.y));
    if (fork < 0)   return false;
    if (fork > 0)   return true;//失败，copy CSDN

}








