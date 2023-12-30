//8.3: cube rotation with any viewing plane. 
#include <iostream>
#include <fstream>
#include<cmath>
#include <vector>
#include <cstring>
#include <sstream>
#include <algorithm> 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

Mat_<double> rotate(double theta, Mat_<double> pts){
    double t = theta;
    Mat xrot = (Mat_<double>(4,4)<<1, 0, 0, 0, 0, cos(t), -sin(t), 0, 0, sin(t), cos(t), 0, 0, 0, 0, 1);
    Mat yrot = (Mat_<double>(4,4)<<cos(t), 0, sin(t), 0, 0, 1, 0, 0, -sin(t), 0, cos(t), 0, 0, 0, 0, 1); 
    Mat zrot = (Mat_<double>(4,4)<<cos(t), -sin(t), 0, 0, sin(t), cos(t), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    return xrot*yrot*zrot*pts;
}
bool planeside(Mat_<double> x, Mat_<double> a, Mat_<double> n, bool cubeside){ //returns true if on the opposite side of plane compared to where the cube is, false if on the same side of the plane as the cube. cubeside true if on left, false if on right
    if((x-a).dot(n)<=0){ //a and x are points, n is technically a line. 
        if(cubeside){
            return false; //sameside
        }
        else{
            return true;
        }
    }
    else{
        if(cubeside){
            return true;
        }
        else{
            return false;
        }
    }
}
Mat_<double> find_pv(Mat_<double> v, Mat_<double> a, Mat_<double> n, Mat_<double> e){ //returns 3d projection of point v on plane defined by a and n, based on eye. 
    double t = ((a-e).dot(n))/((v-e).dot(n));
    Mat Pv = t*(v-e)+e;
    return Pv;
}

Point2d find_uv(Mat_<double> w1, Mat_<double> w2, Mat_<double> P0, Mat_<double> pv){
    Mat p = pv-P0;
    double u = (p.dot(w1)/(w1.dot(w1)));
    double v = (p.dot(w2)/(w2.dot(w2)));
    Point2d uv = Point2d(u+400.0,v+300.0);
    return uv;
}
void part1(int argc, const char* argv[]){
    ofstream f;
    ofstream g;
    ofstream l;
    f.open("coordinates.txt");
    g.open("coordinates2d.txt");
    l.open("log.txt");
    l<<"The plane defined by (x-a)*n=0 is:\n";
    int frame_width = 600;
    Mat plane_a = (Mat_<double>(1,3)<<600, 400, 300);
    Mat plane_n = (Mat_<double>(1,3)<<1,0,0);
    Mat eye = (Mat_<double>(1,3)<<1000,30,90); //check to make sure eye is on non-vertex side of plane 
    
    //command line args for a, n, e
    for(int i =0; i<argc; i++){ 
        if(strcmp(argv[i], "-a") == 0){
            string input = argv[i+1];
            stringstream test(input.substr(1,input.size()-2));
            string segment;
            vector<double> seglist;
            while(getline(test, segment, ',')){
                seglist.push_back(stod(segment));
            }
            plane_a = (Mat_<double>(1,3)<<seglist[0],seglist[1],seglist[2]); 
        }
        else if(strcmp(argv[i], "-n") == 0){
            string input = argv[i+1];
            stringstream test(input.substr(1,input.size()-2));
            string segment;
            vector<double> seglist;
            while(getline(test, segment, ',')){
                seglist.push_back(stod(segment));
            }
            plane_n = (Mat_<double>(1,3)<<seglist[0],seglist[1],seglist[2]); 
        }
        else if(strcmp(argv[i], "-e") == 0){
            string input = argv[i+1];
            stringstream test(input.substr(1,input.size()-2));
            string segment;
            vector<double> seglist;
            while(getline(test, segment, ',')){
                seglist.push_back(stod(segment));
            }
            eye = (Mat_<double>(1,3)<<seglist[0],seglist[1],seglist[2]); 
        }
    }
    l<<"  a = ("<<plane_a.at<double>(0,0)<<","<<plane_a.at<double>(0,1)<<","<<plane_a.at<double>(0,2)<<")\n";
    l<<"  n = ("<<plane_n.at<double>(0,0)<<","<<plane_n.at<double>(0,1)<<","<<plane_n.at<double>(0,2)<<")\n";
    l<<"The eye e is:\n";
    l<<"  e = ("<<eye.at<double>(0,0)<<","<<eye.at<double>(0,1)<<","<<eye.at<double>(0,2)<<")\n";
    l<<"\n";
    Mat points = (Mat_<double>(4,8) << 1,1,1,1,-1,-1,-1,-1,1,1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,-1,1,-1,1,1,1,1,1,1,1,1);
    double t = 0.0; //theta in radians
    int totalframes = 600;
    Mat scale = (Mat_<double>(4,4)<<150.0, 0.0, 0.0, 0.0, 0.0, 150.0, 0.0, 0.0, 0.0, 0.0, 150.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    points = scale*points; //scaling points before rendering
    //multiply all matrices, write frames into vector and then use videowriter to display. 
    //establish w1, p0, and w2 before frames
    Mat origin = (Mat_<double>(1,3)<<0,0,0);
    Mat P0 = find_pv(origin, plane_a, plane_n, eye);
    //getting pv1-3 to find w
    Mat tv2 = (Mat_<double>(1,3)<<points.at<double>(0,0),points.at<double>(1,0), points.at<double>(2,0));
    Mat tv3 = (Mat_<double>(1,3)<<points.at<double>(0,1),points.at<double>(1,1), points.at<double>(2,1));
    Mat tv1 = (Mat_<double>(1,3)<<points.at<double>(0,2),points.at<double>(1,2), points.at<double>(2,2));
    Mat Pv1 = find_pv(tv1, plane_a, plane_n, eye); //3 connected vertices projects
    Mat Pv2 = find_pv(tv2, plane_a, plane_n, eye);
    Mat Pv3 = find_pv(tv3, plane_a, plane_n, eye);
    
    Mat a = Pv3-Pv2; //a, b are lines on the plane
    Mat b = Pv1-Pv2;
//     cout<<"a"<<a.at<double>(0,0)<<" "<<a.at<double>(0,1)<<" "<<a.at<double>(0,2)<<"\n";
//     cout<<"b"<<b.at<double>(0,0)<<" "<<b.at<double>(0,1)<<" "<<b.at<double>(0,2)<<"\n";
    Mat w1 = a/sqrt(a.dot(a));
    Mat w2 = b - (b.dot(a)/a.dot(a))*a;
    w2 = w2/sqrt(w2.dot(w2)); //w1, w2 set for rest of project + rendering
    l<<"Vertices I used to create the 2d coordinate system and their projections are:\n";
    l<<"v1 = ("<<tv1.at<double>(0,0)<<","<<tv1.at<double>(0,1)<<","<<tv1.at<double>(0,2)<<")\n";
    l<<"v2 = ("<<tv2.at<double>(0,0)<<","<<tv2.at<double>(0,1)<<","<<tv2.at<double>(0,2)<<")\n";
    l<<"v3 = ("<<tv3.at<double>(0,0)<<","<<tv3.at<double>(0,1)<<","<<tv3.at<double>(0,2)<<")\n\n";
    l<<"pv1 = ("<<Pv1.at<double>(0,0)<<","<<Pv1.at<double>(0,1)<<","<<Pv1.at<double>(0,2)<<")\n";
    l<<"pv2 = ("<<Pv2.at<double>(0,0)<<","<<Pv2.at<double>(0,1)<<","<<Pv2.at<double>(0,2)<<")\n";
    l<<"pv3 = ("<<Pv3.at<double>(0,0)<<","<<Pv3.at<double>(0,1)<<","<<Pv3.at<double>(0,2)<<")\n\n";
    l<<"The 2 vectors a and b that are in plane are:\n";
    l<<"a = pv3-pv2 = ("<<a.at<double>(0,0)<<","<<a.at<double>(0,1)<<","<<a.at<double>(0,2)<<")\n";
    l<<"b = pv1-pv2 = ("<<b.at<double>(0,0)<<","<<b.at<double>(0,1)<<","<<b.at<double>(0,2)<<")\n\n";
    l<<"The w1 and w2 obtained from a and b are:\n";
    l<<"w1 = ("<<w1.at<double>(0,0)<<","<<w1.at<double>(0,1)<<","<<w1.at<double>(0,2)<<")\n";
    l<<"w2 = ("<<w2.at<double>(0,0)<<","<<w2.at<double>(0,1)<<","<<w2.at<double>(0,2)<<")\n\n";
    l<<"The center of the cube in first frame and its projection are:\n";
    l<<"center = ("<<origin.at<double>(0,0)<<","<<origin.at<double>(0,1)<<","<<origin.at<double>(0,2)<<")\n";
    l<<"p0 = ("<<P0.at<double>(0,0)<<","<<P0.at<double>(0,1)<<","<<P0.at<double>(0,2)<<")\n\n";
    l<<"The coordinates in the 2d plane x = p0 + u*w1 + v*w2 are:\n";
    l<<"  p0 = ("<<P0.at<double>(0,0)<<","<<P0.at<double>(0,1)<<","<<P0.at<double>(0,2)<<")\n";
    l<<"  w1 = ("<<w1.at<double>(0,0)<<","<<w1.at<double>(0,1)<<","<<w1.at<double>(0,2)<<")\n";
    l<<"  w2 = ("<<w2.at<double>(0,0)<<","<<w2.at<double>(0,1)<<","<<w2.at<double>(0,2)<<")\n\n";
    vector<Mat> frames;
    for(int i = 0; i<totalframes/2;i++){
        Mat tpoints = rotate(t,points); 
        vector<Mat_<double>> Pvs; //stores all 8 3d projection of vertices
        for(int k =0;k<8;k++){ //find 3d projections of transformed points
            Mat tpoint = (Mat_<double>(1,3)<<tpoints.at<double>(0,k),tpoints.at<double>(1,k), tpoints.at<double>(2,k));
            Mat Pv = find_pv(tpoint, plane_a, plane_n, eye);
            Pvs.push_back(Pv);
        }
        if(i<4){
            l<<"\n\nThe frame"<<i+1<<" in 3d has the following edges:\n";
            l<<"  ("<<Pvs[0].at<double>(0,0)<<","<<Pvs[0].at<double>(0,1)<<","<<Pvs[0].at<double>(0,2)<<"), ("<<Pvs[1].at<double>(0,0)<<","<<Pvs[1].at<double>(0,1)<<","<<Pvs[1].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[0].at<double>(0,0)<<","<<Pvs[0].at<double>(0,1)<<","<<Pvs[0].at<double>(0,2)<<"), ("<<Pvs[2].at<double>(0,0)<<","<<Pvs[2].at<double>(0,1)<<","<<Pvs[2].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[0].at<double>(0,0)<<","<<Pvs[0].at<double>(0,1)<<","<<Pvs[0].at<double>(0,2)<<"), ("<<Pvs[4].at<double>(0,0)<<","<<Pvs[4].at<double>(0,1)<<","<<Pvs[4].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[1].at<double>(0,0)<<","<<Pvs[1].at<double>(0,1)<<","<<Pvs[1].at<double>(0,2)<<"), ("<<Pvs[3].at<double>(0,0)<<","<<Pvs[3].at<double>(0,1)<<","<<Pvs[3].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[1].at<double>(0,0)<<","<<Pvs[1].at<double>(0,1)<<","<<Pvs[1].at<double>(0,2)<<"), ("<<Pvs[5].at<double>(0,0)<<","<<Pvs[5].at<double>(0,1)<<","<<Pvs[5].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[2].at<double>(0,0)<<","<<Pvs[2].at<double>(0,1)<<","<<Pvs[2].at<double>(0,2)<<"), ("<<Pvs[3].at<double>(0,0)<<","<<Pvs[3].at<double>(0,1)<<","<<Pvs[3].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[2].at<double>(0,0)<<","<<Pvs[2].at<double>(0,1)<<","<<Pvs[2].at<double>(0,2)<<"), ("<<Pvs[6].at<double>(0,0)<<","<<Pvs[6].at<double>(0,1)<<","<<Pvs[6].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[3].at<double>(0,0)<<","<<Pvs[3].at<double>(0,1)<<","<<Pvs[3].at<double>(0,2)<<"), ("<<Pvs[7].at<double>(0,0)<<","<<Pvs[7].at<double>(0,1)<<","<<Pvs[7].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[4].at<double>(0,0)<<","<<Pvs[4].at<double>(0,1)<<","<<Pvs[4].at<double>(0,2)<<"), ("<<Pvs[5].at<double>(0,0)<<","<<Pvs[5].at<double>(0,1)<<","<<Pvs[5].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[4].at<double>(0,0)<<","<<Pvs[4].at<double>(0,1)<<","<<Pvs[4].at<double>(0,2)<<"), ("<<Pvs[6].at<double>(0,0)<<","<<Pvs[6].at<double>(0,1)<<","<<Pvs[6].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[5].at<double>(0,0)<<","<<Pvs[5].at<double>(0,1)<<","<<Pvs[5].at<double>(0,2)<<"), ("<<Pvs[7].at<double>(0,0)<<","<<Pvs[7].at<double>(0,1)<<","<<Pvs[7].at<double>(0,2)<<")\n";
            l<<"  ("<<Pvs[6].at<double>(0,0)<<","<<Pvs[6].at<double>(0,1)<<","<<Pvs[6].at<double>(0,2)<<"), ("<<Pvs[7].at<double>(0,0)<<","<<Pvs[7].at<double>(0,1)<<","<<Pvs[7].at<double>(0,2)<<")\n\n";
            for(int k=0;k<8;k++){
                f<<"("<<Pvs[k].at<double>(0,0)<<","<<Pvs[k].at<double>(0,1)<<","<<Pvs[k].at<double>(0,2)<<"),";
            }
            f<<"\n";
        }
        //transform pv to 2d coords, pv = p0 + u*w1 + v*w2
        vector<Point2d> v; //stores all 8 2d vertices (u,v)
        for(int k =0; k<8;k++){
            Point2d p2d = find_uv(w1, w2, P0, Pvs[k]);
            v.push_back(p2d);
        }
        if(i<4){
            l<<"The frame"<<i+1<<" in 2d has the following edges:\n";
            l<<"  ("<<v[0].x<<","<<v[0].y<<"), ("<<v[1].x<<","<<v[1].y<<")\n";
            l<<"  ("<<v[0].x<<","<<v[0].y<<"), ("<<v[2].x<<","<<v[2].y<<")\n";
            l<<"  ("<<v[0].x<<","<<v[0].y<<"), ("<<v[4].x<<","<<v[4].y<<")\n";
            l<<"  ("<<v[1].x<<","<<v[1].y<<"), ("<<v[3].x<<","<<v[3].y<<")\n";
            l<<"  ("<<v[1].x<<","<<v[1].y<<"), ("<<v[5].x<<","<<v[5].y<<")\n";
            l<<"  ("<<v[2].x<<","<<v[2].y<<"), ("<<v[3].x<<","<<v[3].y<<")\n";
            l<<"  ("<<v[2].x<<","<<v[2].y<<"), ("<<v[6].x<<","<<v[6].y<<")\n";
            l<<"  ("<<v[3].x<<","<<v[3].y<<"), ("<<v[7].x<<","<<v[7].y<<")\n";
            l<<"  ("<<v[4].x<<","<<v[4].y<<"), ("<<v[5].x<<","<<v[5].y<<")\n";
            l<<"  ("<<v[4].x<<","<<v[4].y<<"), ("<<v[6].x<<","<<v[6].y<<")\n";
            l<<"  ("<<v[5].x<<","<<v[5].y<<"), ("<<v[7].x<<","<<v[7].y<<")\n";
            l<<"  ("<<v[6].x<<","<<v[6].y<<"), ("<<v[7].x<<","<<v[7].y<<")";
            for(int k=0;k<8;k++){
                g<<"("<<v[k].x<<","<<v[k].y<<"),";
            }
            g<<"\n";
        }
        Mat frame = Mat::zeros(600, 800, CV_8UC3); //nrows, ncols aka height, width
        line(frame, v[0], v[1], Scalar(255, 255, 255), 5, LINE_8); //5 is thickness of line 
        line(frame, v[0], v[2], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[0], v[4], Scalar(255, 255, 255), 5, LINE_8);
        line(frame, v[1], v[3], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[1], v[5], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[2], v[3], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[2], v[6], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[3], v[7], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[4], v[5], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[4], v[6], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[5], v[7], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[6], v[7], Scalar(255, 255, 255), 5, LINE_8); 
        for(int k =0; k<8;k++){ //rendering, draw the vertices
            circle(frame,v[k],10,Scalar(0, 0, 255),-1, LINE_8); //-1 means filled 
        }
        frames.push_back(frame); //add frame to animation. 
        t+=0.02;
    }
    for(int i = 0; i<totalframes/2;i++){
        Mat tpoints = rotate(t,points); 
        vector<Mat_<double>> Pvs; //stores all 8 3d projection of vertices
        for(int k =0;k<8;k++){ //find 3d projections of transformed points
            Mat tpoint = (Mat_<double>(1,3)<<tpoints.at<double>(0,k),tpoints.at<double>(1,k), tpoints.at<double>(2,k));
            Mat Pv = find_pv(tpoint, plane_a, plane_n, eye);
            Pvs.push_back(Pv);
        }
        //transform pv to 2d coords, pv = p0 + u*w1 + v*w2
        vector<Point2d> v; //stores all 8 2d vertices (u,v)
        for(int k =0; k<8;k++){
            Point2d p2d = find_uv(w1, w2, P0, Pvs[k]);
           // cout<<"u "<<p2d.x<<" v "<<p2d.y<<"\n";
            v.push_back(p2d);
        }
        
        Mat frame = Mat::zeros(600, 800, CV_8UC3); //nrows, ncols aka height, width
        line(frame, v[0], v[1], Scalar(255, 255, 255), 5, LINE_8); //5 is thickness of line 
        line(frame, v[0], v[2], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[0], v[4], Scalar(255, 255, 255), 5, LINE_8);
        line(frame, v[2], v[4], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[1], v[2], Scalar(255, 255, 255), 5, LINE_8); 
        line(frame, v[1], v[4], Scalar(255, 255, 255), 5, LINE_8); 
        circle(frame,v[0],10,Scalar(0, 0, 255),-1, LINE_8); //-1 means filled 
        circle(frame,v[1],10,Scalar(0, 0, 255),-1, LINE_8);
        circle(frame,v[2],10,Scalar(0, 0, 255),-1, LINE_8);
        circle(frame,v[4],10,Scalar(0, 0, 255),-1, LINE_8);
        frames.push_back(frame); //add frame to animation. 
        t+=0.02;
    }
    //videowriter
    VideoWriter out("rotation.avi",VideoWriter::fourcc('M','J','P','G'), 10, Size(800, 600));
    for(Mat i : frames){
        out.write(i);
    }
    out.release();
    f.close();
    g.close();
    l.close();
}
int main(int argc, const char* argv[]){
    part1(argc, argv);
}