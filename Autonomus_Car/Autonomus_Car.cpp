#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

//image processing variables
Mat frame, Matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate, frameFinalDuplicate1;
Mat ROILane, ROILaneEnd;
int LeftLanePos,RightLanePos, frameCenter, lanecenter, Result,laneEnd;

RaspiCam_Cv Camera;

stringstream ss;

vector<int> histrogramLane;
vector<int> histrogramLaneEnd;

int width = 580;
int height = 240;
int Centre = 290;

int roi_value = 240; // region of interest height
int pers_iter = 60; 

Point2f Source[] = {Point2f(70,(height - 80)),Point2f((width-90),(height - 80)),Point2f(25,(height - 30)),Point2f((width-50),(height - 30))};
Point2f Destination[] = {Point2f(pers_iter,0),Point2f((width-pers_iter),0),Point2f(pers_iter,height),Point2f((width-pers_iter),height)};

// machine learning variables
CascadeClassifier Stop_Cascade,Object_Cascade,Light_Cascade;
Mat frame_Stop, RoI_Stop, gray_Stop,frame_Object, RoI_Object, gray_Object;
Mat frame_Light ,RoI_Light, gray_Light;

vector<Rect> Stop,Object,Light;

int dist_stop = 0;
int dist_Object = 0;
int dist_Light = 0;

void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,width ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,height ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,100));

}
void Capture()
{
	  Camera.grab();
    Camera.retrieve( frame);
    cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
    cvtColor(frame, frame_Object, COLOR_BGR2RGB);
    cvtColor(frame, frame_Light, COLOR_BGR2RGB);
    cvtColor(frame, frame, COLOR_BGR2RGB);
}
void Perspective()
{
  line(frame,Source[0],Source[1],Scalar(0,0,255),2);
  line(frame,Source[1],Source[3],Scalar(0,0,255),2);
  line(frame,Source[3],Source[2],Scalar(0,0,255),2);
  line(frame,Source[2],Source[0],Scalar(0,0,255),2);

  Matrix = getPerspectiveTransform(Source,Destination);
  warpPerspective(frame,framePers, Matrix,Size(width,height));
}
void Threshold()
{
  cvtColor(framePers,frameGray,COLOR_RGB2GRAY);
  inRange(frameGray,70,255,frameThresh);
  Canny(frameGray, frameEdge, 120, 165, 3, false );
  add(frameThresh, frameEdge, frameFinal);
  cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
  cvtColor(frameFinal, frameFinalDuplicate,COLOR_RGB2BGR);
  cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR);
}
void Histrogram()
{
  histrogramLane.resize(width);
  histrogramLane.clear();

  for(int i=0; i<width; i++)
  {
    ROILane= frameFinalDuplicate(Rect(i,(height-roi_value),1,roi_value));
    divide(255,ROILane,ROILane);
    histrogramLane.push_back((int)(sum(ROILane)[0]));
  }

  ////////////   ENDLANE   //////////
  histrogramLaneEnd.resize(width);
  histrogramLaneEnd.clear();
	for (int i = 0; i < width; i++)       
	{
		ROILaneEnd = frameFinalDuplicate1(Rect(i, 0, 1, height));   
		divide(255, ROILaneEnd, ROILaneEnd);       
		histrogramLaneEnd.push_back((int)(sum(ROILaneEnd)[0]));  
		
	
	}
	laneEnd = sum(histrogramLaneEnd)[0];
}
void LaneFinder()
{
  vector<int>:: iterator LeftPtr;
  LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 210);
  LeftLanePos = distance(histrogramLane.begin(), LeftPtr);

  vector<int>:: iterator RightPtr;
  RightPtr = max_element(histrogramLane.begin() + 370 , histrogramLane.end());
  RightLanePos = distance(histrogramLane.begin(), RightPtr);

  line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, height), Scalar(0, 255, 0), 2);
  line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, height), Scalar(0, 255, 0), 2);
}
void LaneCenter()
{
  lanecenter = (RightLanePos-LeftLanePos)/2 + LeftLanePos;
  frameCenter = Centre;

  line(frameFinal, Point2f(lanecenter, 0), Point2f(lanecenter, height), Scalar(0, 255, 0), 3);
  line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, height), Scalar(255, 0, 0), 3);

  Result= lanecenter - frameCenter;
 
}
void PrintScreen()
{
    ss.str(" ");
    ss.clear();
    ss<<"Result = "<<Result;
    putText(frame, ss.str(), Point2f(1, 50), 0,1, Scalar(0, 0, 255), 2);

    namedWindow("original",WINDOW_KEEPRATIO);
    moveWindow("original",50,50);
    resizeWindow("original", 400, height);
    imshow("original", frame);


    namedWindow("final",WINDOW_KEEPRATIO);
    moveWindow("final",460,50); 
    resizeWindow("final", 400, height);
    imshow("final", frameFinal);

    namedWindow("stop",WINDOW_KEEPRATIO);
    moveWindow("stop",50,(height+50));
    resizeWindow("stop", 400, height);
    imshow("stop", RoI_Stop);

    namedWindow("light",WINDOW_KEEPRATIO);
    moveWindow("light",460,(height+50));
    resizeWindow("light", 400, height);
    imshow("light", RoI_Light);

}
void Stop_detection(){

  if(!Stop_Cascade.load("/home/pi/Desktop/Autonomus_Car/stop_cascade.xml")){
    cout<<"unable to open stop cascade file"<<endl;
  }
    RoI_Stop = frame_Stop(Rect(300,0,280,240));

    cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(gray_Stop, gray_Stop);
    Stop_Cascade.detectMultiScale(gray_Stop, Stop);
  for(int i=0; i<Stop.size(); i++)
    {
	Point P1(Stop[i].x, Stop[i].y);
	Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);
	
	rectangle(RoI_Stop, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Stop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);

  dist_stop = (-1)*(P2.x-P1.x)+(78);

  ss.str(" ");
  ss.clear();
  ss<<"D = "<<dist_stop<<" cm";
  putText(RoI_Stop, ss.str(), Point2f(70,150), 0,0.7, Scalar(0,0,255), 2);

    }
}
void Object_detection(){

  if(!Object_Cascade.load("/home/pi/Desktop/Autonomus_Car/object_cascade.xml")){
    cout<<"unable to open object cascade file"<<endl;
  }
    RoI_Object = frame_Object(Rect(0,0,580,240));

    cvtColor(RoI_Object, gray_Object, COLOR_RGB2GRAY);
    equalizeHist(gray_Object, gray_Object);
    Object_Cascade.detectMultiScale(gray_Object, Object);
  for(int i=0; i<Object.size(); i++)
    {
	Point P1(Object[i].x, Object[i].y);
	Point P2(Object[i].x + Object[i].width, Object[i].y + Object[i].height);
	
	rectangle(RoI_Object, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Object, "Car", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);

  dist_Object = (-0.5)*(P2.x-P1.x)+(112.5);

  ss.str(" ");
  ss.clear();
  ss<<"D = "<<dist_Object<<" cm";
  putText(RoI_Object, ss.str(), Point2f(70,150), 0,0.7, Scalar(0,0,255), 2);

    }
}
void Light_detection(){

  if(!Light_Cascade.load("/home/pi/Desktop/Autonomus_Car/light_cascade.xml")){
    cout<<"unable to open Light cascade file"<<endl;
  }
    RoI_Light = frame_Light(Rect(0,0,580,240));

    cvtColor(RoI_Light, gray_Light, COLOR_RGB2GRAY);
    equalizeHist(gray_Light, gray_Light);
    Light_Cascade.detectMultiScale(gray_Light, Light);
  for(int i=0; i<Light.size(); i++)
    {
	Point P1(Light[i].x, Light[i].y);
	Point P2(Light[i].x + Light[i].width, Light[i].y + Light[i].height);
	
	rectangle(RoI_Light, P1, P2, Scalar(0, 0, 255), 2);
	putText(RoI_Light, "Red Light", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);

  dist_Light = (-0.769)*(P2.x-P1.x)+(77.681);

  ss.str(" ");
  ss.clear();
  ss<<"D = "<<dist_Light<<" cm";
  putText(RoI_Light, ss.str(), Point2f(70,150), 0,0.7, Scalar(0,0,255), 2);

    }
}
int main(int argc,char **argv)
{
  wiringPiSetup();
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);

	Setup(argc, argv, Camera);

	if (!Camera.open())
	{
	cout<<"Failed to Connect"<<endl;
  }
     
    while(1)
    {
    auto start = std::chrono::system_clock::now();

    Capture();
    Perspective();
    Threshold();
    Histrogram();
    LaneFinder();
    LaneCenter();
    Stop_detection();
    //Object_detection();
    Light_detection();

    cout<<laneEnd<<endl;

    if(dist_Light > 10 && dist_Light < 45){
  digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 10
	digitalWrite(23, 0);
	digitalWrite(24, 1);
	cout<<"Light"<<endl;
  dist_Light = 0;
  goto Light;
    }
  //   if(dist_Object > 10 && dist_Object < 35){
  // digitalWrite(21, 1);
	// digitalWrite(22, 0);    //decimal = 9
	// digitalWrite(23, 0);
	// digitalWrite(24, 1);
	// cout<<"Car"<<endl;
  // dist_Object = 0;
  // goto Object;
  //   }
    if(dist_stop > 25 && dist_stop < 35){
  digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 8
	digitalWrite(23, 0);
	digitalWrite(24, 1);
	cout<<"Stop sign"<<endl;
  dist_stop = 0;
  goto stop_sign;
    }
    if (laneEnd > 60000 )
    {
  digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 7
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Lane End"<<endl;
  goto End;
    }
    if (Result >-15 && Result <15)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 0
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Forward"<<endl;
    }
    else if (Result >=15 && Result <45)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 1
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right1"<<endl;
    }
    else if (Result >=45 && Result <65)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 2
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right2"<<endl;
    }
    else if (Result >=65)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 1);    //decimal = 3
	digitalWrite(23, 0);
	digitalWrite(24, 0);
	cout<<"Right3"<<endl;
    }
    else if (Result <=-15 && Result >-45)
    {
	digitalWrite(21, 0);
	digitalWrite(22, 0);    //decimal = 4
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left1"<<endl;
    }
    else if (Result <=-45 && Result >-65)
    {
	digitalWrite(21, 1);
	digitalWrite(22, 0);    //decimal = 5
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left2"<<endl;
    }
    else if (Result <=-65 )
    {
	digitalWrite(21, 0);
	digitalWrite(22, 1);    //decimal = 6
	digitalWrite(23, 1);
	digitalWrite(24, 0);
	cout<<"Left3"<<endl;
    }

  stop_sign:
  Object:
  End:
  Light:
  
  PrintScreen();

  waitKey(1);
    
  }
    
  return 0;
     
}
