#include"define.h"
#include"detecte.h"
#include"anti_gyro.h"

using namespace std;
using namespace cv;

cv::Mat img;
cv::Mat Aim;
cv::Mat gray;
std::vector<std::vector<cv::Point>> contours;
std::vector<cv::RotatedRect> FitLightbar;
std::vector<cv::Vec4i> hierarchy;
cv::VideoCapture video;

bool if_play = true;
int tot_frame;

int frame = 0;

void onMouseChange(int event, int x, int y, int flags, void* param);
void onFrameChange(int pos, void*);
void Resort_Libar_Point(vector<Point2d> Libar);
void Resort_Armo_Point(vector<Point2d> Armo);

void video_detect()
{
    namedWindow("mark",0);
    namedWindow("Aim",0);
    namedWindow("two",0);
    //Mat channels[3];
    vector<Mat> channel;
   // vector<Vec3f> circles;
    //video.set(CV_CAP_PROP_EXPOSURE,10);
    bool flag = false;
    bool anti = false;
    video.open(V2);
    tot_frame = (int)video.get(CAP_PROP_FRAME_COUNT);
    setMouseCallback("mark", onMouseChange);
    createTrackbar("process", "mark", &frame, tot_frame, onFrameChange);
    //video.open(0);
    if (video.isOpened())
    {
        while (1)
        {
            
            //namedWindow("mark");
            video >> img;
            //cvtColor(img, gray, CV_BGR2GRAY);
            split(img, channel);
            threshold(channel[2], channel[2],110, 255, 0);
            //Aim = gray - 1.0 * channel[0] + 0.7 * channel[2] - 1.0 * channel[1]+0.15*channel[2];///红色专用！！！

            Aim = channel[2] -1.0 * channel[0] - 0.8 * channel[1];

            //threshold(Aim, Aim, 200, 255, 0);///!!!!!!!!!!!!!蓝红//蓝色笔红色好
            //Aim = 1.2*channel[1]-1.05*channel[0]+0.6*channel[2];
            //threshold(Aim, Aim, 70, 255, 0);

            Mat element1 = getStructuringElement(MORPH_RECT, Size(2, 2));
            Mat element2 = getStructuringElement(MORPH_RECT, Size(3, 3));

            dilate(Aim, Aim, element1);
            erode(Aim, Aim, element2);
            dilate(Aim, Aim, element2);
            //dilate(Aim, Aim, element2);

            findContours(Aim, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

            for (const auto& contour : contours)
            {
                if (contour.size() < 60) continue;
                RotatedRect rec = minAreaRect(contour);
                RotatedRect ell = fitEllipse(contour);
                Point2f* vertex = new Point2f[4];//顶点
                ell.points(vertex);
                float Xdiff1 = vertex[0].x - vertex[1].x;
                float Ydiff1 = vertex[0].y - vertex[1].y;
                float SideLength1 = sqrt(Xdiff1 * Xdiff1 + Ydiff1 * Ydiff1);
                float Xdiff2 = vertex[0].x - vertex[3].x;
                float Ydiff2 = vertex[0].y - vertex[3].y;
                float SideLength2 = sqrt(Xdiff2 * Xdiff2 + Ydiff2 * Ydiff2);
                //if(abs(ell.angle)>30) continue;
                if(SideLength1>=SideLength2)
                {
                    float LenRatio = SideLength1 / SideLength2;
                    if(LenRatio>1.2/*&&LenRatio<5*/)
                    {
                        ellipse(img, ell, Scalar(255, 255, 255));
                        FitLightbar.push_back(ell);
                    }
                }

            }


            Point2f FTop0, FBot0, STop0, SBot0;
            vector<Point2d> armo;
            int sum_arm=0;
            for (size_t i = 0; (i + 1) < FitLightbar.size(); i++)
            {
                for (size_t j = 0; j < 2; j++)
                {
                    if (i + j + 1 == FitLightbar.size()) break;

                    RotatedRect& FirstLibar = FitLightbar[i];
                    RotatedRect& SecondLibar = FitLightbar[i + j + 1];
                    Point2f FirstPoint[4], SecondPoint[4];
                    FirstLibar.points(FirstPoint);
                    SecondLibar.points(SecondPoint);

                    Point2f FTop, FBot, STop, SBot;

                    FTop.x = (FirstPoint[1].x + FirstPoint[2].x) / 2;
                    FTop.y = (FirstPoint[1].y + FirstPoint[2].y) / 2;
                    FBot.x = (FirstPoint[0].x + FirstPoint[3].x) / 2;
                    FBot.y = (FirstPoint[0].y + FirstPoint[3].y) / 2;
                    STop.x = (SecondPoint[1].x + SecondPoint[2].x) / 2;
                    STop.y = (SecondPoint[1].y + SecondPoint[2].y) / 2;
                    SBot.x = (SecondPoint[0].x + SecondPoint[3].x) / 2;
                    SBot.y = (SecondPoint[0].y + SecondPoint[3].y) / 2;

                    if (FTop.y > FBot.y)
                    {
                        Point2d Temp1 = FTop;
                        FTop = FBot;
                        FBot = Temp1;
                    }

                    if (STop.y > SBot.y)
                    {
                        Point2d Temp2 = STop;
                        STop = SBot;
                        SBot = Temp2;
                    }

                    if (FTop.x > STop.x)
                    {
                        Point2d temp1, temp2;
                        temp1 = FTop;
                        temp2 = FBot;
                        FTop = STop;
                        STop = temp1;
                        FBot = SBot;
                        SBot = temp2;
                    }

                    //Libar.push_back(FTop);
                    //Libar.push_back(FBot);
                    //Libar.push_back(STop);
                    //Libar.push_back(SBot);



                    float FirdeltaY = FTop.y - FBot.y;
                    float FirdeltaX = FTop.x - FBot.x;
                    float SecdeltaY = STop.y - SBot.y;
                    float SecdeltaX = STop.x - SBot.x;
                    float len1 = sqrt((FTop.x - FBot.x) * (FTop.x - FBot.x) + (FTop.y - FBot.y) * (FTop.y - FBot.y));
                    float len2 = sqrt((STop.x - SBot.x) * (STop.x - SBot.x) + (STop.y - SBot.y) * (STop.y - SBot.y));
                    //std::cout<<"len1:"<<len1<<"\t"<<"len2:"<<len2<<"\n";
                    float dis1=sqrt((FTop.x - STop.x) * (FTop.x - STop.x) + (FTop.y - STop.y) * (FTop.y - STop.y));
                    float dis2=sqrt((FBot.x - SBot.x) * (FBot.x - SBot.x) + (FBot.y - SBot.y) * (FBot.y - SBot.y));
                    //std::cout<<"dis1:"<<dis1<<"\t"<<"dis2:"<<dis2<<"\n";
                    float len=(len1+len2)/2;
                    float dis=(dis1+dis2)/2;

                    if (abs(FTop.y - STop.y) > (len / 2.25))//  height//1.7
                    {
                        string siftm1 = "FT.y-ST.y  ";
                        string ma1 = to_string(abs(FTop.y - STop.y));
                        string nu1 = to_string(len);
                        string spa1 = "  1  ";
                        string fin1 = siftm1 + ma1 + spa1 + nu1;
                        Point2d Mid1;
                        Mid1.x = (FBot.x + SBot.x) / 2;
                        Mid1.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin1, Mid1, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }

                        
                    /*if (abs(FBot.y-SBot.y) > (len / 1.75))
                    {
                        string siftm2 = "FB.y-SB.y  ";
                        string ma2 = to_string(abs(FTop.y - STop.y));
                        string nu2 = to_string(len);
                        string spa2 = "  2  ";
                        string fin2 = siftm2 + ma2 + spa2 + nu2;
                        Point2d Mid2;
                        Mid2.x = (FBot.x + SBot.x) / 2;
                        Mid2.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin2, Mid2, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }*/
                    if (abs(FTop.x - FBot.x) > (dis / 2))//  flat
                    {
                        string siftm3 = "FT.x-FB.x  ";
                        string ma3 = to_string(abs(FTop.y - STop.y));
                        string nu3 = to_string(dis);
                        string spa3 = "  3  ";
                        string fin3 = siftm3 + ma3 + spa3 + nu3;
                        Point2d Mid3;
                        Mid3.x = (FBot.x + SBot.x) / 2;
                        Mid3.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin3, Mid3, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    
                    if (abs(STop.x-SBot.x)>(dis/2))
                    {
                        string siftm4 = "ST.x-SB.x  ";
                        string ma4 = to_string(abs(FTop.y - STop.y));
                        string nu4 = to_string(dis);
                        string spa4 = "  4  ";
                        string fin4 = siftm4 + ma4 + spa4 + nu4;
                        Point2d Mid4;
                        Mid4.x = (FBot.x + SBot.x) / 2;
                        Mid4.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin4, Mid4, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    if (abs(FTop.x-STop.x)>(dis*1.5))
                    {
                        string siftm5 = "FT.x-ST.x  ";
                        string ma5 = to_string(abs(FTop.y - STop.y));
                        string nu5 = to_string(dis);
                        string spa5 = "  5  ";
                        string fin5 = siftm5 + ma5 + spa5 + nu5;
                        Point2d Mid5;
                        Mid5.x = (FBot.x + SBot.x) / 2;
                        Mid5.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin5, Mid5, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    if (abs(SBot.x-FBot.x)>(dis*1.5))
                    {
                        string siftm6 = "SB.x-FB.x  ";
                        string ma6 = to_string(abs(FTop.y - STop.y));
                        string nu6 = to_string(dis);
                        string spa6 = "  6  ";
                        string fin6 = siftm6 + ma6 + spa6 + nu6;
                        Point2d Mid6;
                        Mid6.x = (FBot.x + SBot.x) / 2;
                        Mid6.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin6, Mid6, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    if (abs(FTop.y - SBot.y) < (len / 3))//  height
                    {
                        string siftm7 = "FT.y-SB.y  ";
                        string ma7 = to_string(abs(FTop.y - STop.y));
                        string nu7 = to_string(dis);
                        string spa7 = "  7  ";
                        string fin7 = siftm7 + ma7 + spa7 + nu7;
                        Point2d Mid7;
                        Mid7.x = (FBot.x + SBot.x) / 2;
                        Mid7.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin7, Mid7, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                        
                    if (abs(FBot.y-STop.y)<(len/3))
                    {
                        string siftm8 = "FB.y-ST.y  ";
                        string ma8 = to_string(abs(FTop.y - STop.y));
                        string nu8 = to_string(dis);
                        string spa8 = "  8  ";
                        string fin8 = siftm8 + ma8 + spa8 + nu8;
                        Point2d Mid8;
                        Mid8.x = (FBot.x + SBot.x) / 2;
                        Mid8.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin8, Mid8, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    if (abs(FTop.y-SBot.y)>(len*1.5))
                    {
                        string siftm9 = "FT.y-SB.y  ";
                        string ma9 = to_string(abs(FTop.y - STop.y));
                        string nu9 = to_string(dis);
                        string spa9 = "  9  ";
                        string fin9 = siftm9 + ma9 + spa9 + nu9;
                        Point2d Mid9;
                        Mid9.x = (FBot.x + SBot.x) / 2;
                        Mid9.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin9, Mid9, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    if (abs(FBot.y-STop.y)>(len*1.5))
                    {
                        string siftm10 = "FT.y-SB.y  ";
                        string ma10 = to_string(abs(FTop.y - STop.y));
                        string nu10 = to_string(dis);
                        string spa10 = "  10  ";
                        string fin10 = siftm10 + ma10 + spa10 + nu10;
                        Point2d Mid10;
                        Mid10.x = (FBot.x + SBot.x) / 2;
                        Mid10.y = (FBot.y + SBot.y) / 2;
                        line(img, FBot, SBot, Scalar(255, 255, 255));
                        putText(img, fin10, Mid10, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                        continue;
                    }
                    float FirSlope = FirdeltaY / FirdeltaX;
                    float SecSlope = SecdeltaY / SecdeltaX;
                    float FirInvSlope = FirdeltaX / FirdeltaY;
                    float SecInvSlope = SecdeltaX / SecdeltaY;

                    if (abs(FirInvSlope - SecInvSlope) < 1.01)// 原初1.5
                    {

                        if ((len1 / len2) > 1.4 || (len1 / len2) < 0.6)
                        {
                            string siftmf1 = "len1/len2  ";
                            string maf1 = to_string(len1 / len2);
                            string spaf1 = "f1  ";
                            string finf1 = siftmf1 + spaf1 + maf1;
                            Point2d Midf1;
                            Midf1.x = (FBot.x + SBot.x) / 2;
                            Midf1.y = (FBot.y + SBot.y) / 2;
                            line(img, FBot, SBot, Scalar(255, 255, 255));
                            putText(img, finf1, Midf1, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                            continue;
                        }
                            
                        if (dis / len < 1.2)
                        {
                            string siftmf2 = "dis/len  ";
                            string maf2 = to_string(dis / len);
                            string spaf2 = "f2  ";
                            string finf2 = siftmf2 + spaf2 + maf2;
                            Point2d Midf2;
                            Midf2.x = (FBot.x + SBot.x) / 2;
                            Midf2.y = (FBot.y + SBot.y) / 2;
                            line(img, FBot, SBot, Scalar(255, 255, 255));
                            putText(img, finf2, Midf2, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                            continue;
                        }
                            
                        if(dis/len>2.8) //2.7
                        {
                            string siftmf3 = "dis/len  ";
                            string maf3 = to_string(dis / len);
                            string spaf3 = "f3  ";
                            string finf3 = siftmf3 + spaf3 + maf3;
                            Point2d Midf3;
                            Midf3.x = (FBot.x + SBot.x) / 2;
                            Midf3.y = (FBot.y + SBot.y) / 2;
                            line(img, FBot, SBot, Scalar(255, 255, 255));
                            putText(img, finf3, Midf3, FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8);
                            continue;
                            continue;
                        }
                        flag = true;
                        if (flag)
                        {
                            sum_arm++;
                            FTop0 = FTop;
                            FBot0 = FBot;
                            SBot0 = SBot;
                            STop0 = STop;
                            armo.push_back(FTop0);
                            armo.push_back(FBot0);
                            armo.push_back(STop0);
                            armo.push_back(SBot0);

                            if(sum_arm==2)  break;

                        }
                    }


//                    if (abs(FirSlope - SecSlope) < 1.5)
//                    {
//                        if ((len1 / len2) > 1.35 || (len1 / len2) < 0.7) continue;
//                        if(FirSlope>100||SecSlope>100)  continue;
//                        if(dis/len<1.5) continue;
//                        if(dis/len>3.8) continue;
//                        flag = true;
//                        if (flag)
//                        {
//                            FTop0 = FTop;
//                            FBot0 = FBot;
//                            SBot0 = SBot;
//                            STop0 = STop;
//                            line(img, FTop0, STop0, Scalar(255, 255, 255),3);
//                            line(img, FTop0, FBot0, Scalar(255, 255, 255),3);
//                            line(img, SBot0, FBot0, Scalar(255, 255, 255),3);
//                            line(img, SBot0, STop0, Scalar(255, 255, 255),3);
//                            circle(img, FTop0, 6, Scalar(0, 255, 0), 3, 8);
//                            circle(img, FBot0, 8, Scalar(0, 255, 0), 3, 8);
//                            circle(img, SBot0, 10, Scalar(0, 255, 0), 3, 8);
//                            circle(img, STop0, 12, Scalar(0, 255, 0), 3, 8);
//                            //cout << FTop.x << " " << FTop.y << "  " << FBot.x << " " << FBot.y << "  " << STop.x << " " << STop.y << "  " << SBot.x << " " << SBot.y<<"\n";
//                            //break;
//                        }

//                    }

                }
                if(sum_arm==2)  break;
            }
            if (flag)
            {
                if(sum_arm==1)
                {
                    vector<Point2d> am;
                    Point2d af[4];
                    am.push_back(FTop0);
                    am.push_back(FBot0);
                    am.push_back(STop0);
                    am.push_back(SBot0);


                    af[0] = FTop0;
                    af[1] = STop0;
                    af[2] = FBot0;
                    af[3] = SBot0;

                    /*double X = 0, Y = 0;

                    for (int i = 0; i < 4; i++)
                    {
                        X += am[i].x;
                        Y += am[i].y;
                    }

                    Point2d wei_center(X/4,Y/4);

                    for(int i=0;i<4;i++)
                    {
                        if(am[i].x<wei_center.x&&am[i].y<wei_center.y)
                            af[0]=am[i];
                        if(am[i].x>wei_center.x&&am[i].y<wei_center.y)
                            af[1]=am[i];
                        if(am[i].x<wei_center.x&&am[i].y>wei_center.y)
                            af[2]=am[i];
                        if(am[i].x>wei_center.x&&am[i].y>wei_center.y)
                            af[3]=am[i];
                    }*/

                    line(img, af[0], af[1], Scalar(255, 255, 255),2);
                    line(img, af[0], af[2], Scalar(255, 255, 255),2);
                    line(img, af[3], af[1], Scalar(255, 255, 255),2);
                    line(img, af[3], af[2], Scalar(255, 255, 255),2);
                    circle(img, af[0], 6, Scalar(0, 255, 0), 2, 8);
                    circle(img, af[1], 8, Scalar(0, 255, 0), 2, 8);
                    circle(img, af[2], 10, Scalar(0, 255, 0), 2, 8);
                    circle(img, af[3], 12, Scalar(0, 255, 0), 2, 8);

                    sum_arm=0;
                }
                flag = false;
                if(sum_arm==2)
                {
                    anti=true;
                    sum_arm=0;
                    vector<Point2d> arm;
                    arm=solve_unvisual(armo);
#if DEBUG
                    line(img, arm[0], arm[1], Scalar(255, 255, 255),2);
                    line(img, arm[0], arm[2], Scalar(255, 255, 255),2);
                    line(img, arm[3], arm[1], Scalar(255, 255, 255),2);
                    line(img, arm[3], arm[2], Scalar(255, 255, 255),2);
                    circle(img, arm[0], 6, Scalar(0, 255, 0), 2, 8);
                    circle(img, arm[1], 8, Scalar(0, 255, 0), 2, 8);
                    circle(img, arm[2], 10, Scalar(0, 255, 0), 2, 8);
                    circle(img, arm[3], 12, Scalar(0, 255, 0), 2, 8);
                    line(img, arm[4], arm[5], Scalar(255, 255, 255),2);
                    line(img, arm[4], arm[6], Scalar(255, 255, 255),2);
                    line(img, arm[7], arm[5], Scalar(255, 255, 255),2);
                    line(img, arm[7], arm[6], Scalar(255, 255, 255),2);
                    circle(img, arm[4], 6, Scalar(0, 255, 255), 2, 8);
                    circle(img, arm[5], 8, Scalar(0, 255, 255), 2, 8);
                    circle(img, arm[6], 10, Scalar(0, 255, 255), 2, 8);
                    circle(img, arm[7], 12, Scalar(0, 255, 255), 2, 8);
#endif
                    imshow("two",img);
                    waitKey(10);
                }
            }
            FitLightbar.clear();
            armo.clear();
            if (if_play)
            {
                imshow("mark", img);
                waitKey(1);
                imshow("Aim", Aim);
                waitKey(1);
            }
            else
            {
                imshow("mark", img);
                waitKey(0);
                imshow("Aim", Aim);
                waitKey(0);
            }
            
        }
    }
    return;
}

void onMouseChange(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
        if (if_play) if_play = false;
        else
        {
            if_play = true;
        }
    }
}

void onFrameChange(int pos, void*)
{
    video.set(CAP_PROP_POS_FRAMES, pos);
}


void Resort_Libar_Point(vector<Point2d> Libar)
{

}

void Resort_Armo_Point(vector<Point2d> Armo)
{

}