#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <ctime>
#include <wiringPi.h>

#define TRIG 4
#define ECHO 5
int measure =10;


using namespace std;
using namespace cv;
using namespace raspicam;
Mat frame, Matrix, framePerspect, frameGray, frameThershold, frameEdge, frameFinal,frameFinalDuplicate;
Mat RIOLane;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, offsett;
RaspiCam_Cv  Camera;

stringstream ss;

vector<int>histogramLane;

Point2f Source[] = {Point2f(40,160), Point2f(356,160), Point2f(0,220), Point2f(400,220)};
Point2f Source1[] = {Point2f(100,0), Point2f(270,0), Point2f(100,240), Point2f(270,240)};

//ML Variables

CascadeClassifier  Stop_Cascade, Traffic_Cascade, Pedestrian_Cascade;
Mat  frame_Stop, ROI_Stop, Gray_Stop,frame_Traffic, ROI_Traffic, Gray_Traffic, frame_Pedestrian, ROI_Pedestrian, Gray_Pedestrian;
vector<Rect>Stop, Traffic, Pedestrian;
int distance_stop, distance_Traffic, distance_Pedestrian;


 void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}
void Perspective()
{
    line(frame, Source[0], Source[1], Scalar(255,0,0), 2);
    line(frame, Source[1], Source[3], Scalar(255,0,0), 2);
    line(frame, Source[3], Source[2], Scalar(255,0,0), 2);
    line(frame, Source[2], Source[0], Scalar(255,0,0), 2);
    
    
    
    Matrix = getPerspectiveTransform( Source, Source1);
    warpPerspective(frame, framePerspect, Matrix, Size(400,240)); 
}
void threshold()
{
    cvtColor(framePerspect,frameGray,COLOR_RGB2GRAY);
    inRange(frameGray, 240, 255,frameThershold);
    Canny(frameGray ,frameEdge, 450, 500, 3, false);
    add(frameThershold, frameEdge, frameFinal);
    cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
    cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);//used in histrogram only
}
void histogram()
{
    histogramLane.resize(400);
    histogramLane.clear(); 
    for(int i=0; i< frame.size().width;i++)
    {
        RIOLane = frameFinalDuplicate(Rect(i,140,1,100));
        divide(255 ,RIOLane ,RIOLane );
        histogramLane.push_back((int)(sum(RIOLane)[0]));
    }   
}
void LaneFinder(){
    vector<int>:: iterator LeftPtr;
    LeftPtr = max_element(histogramLane.begin(), histogramLane.begin() +150);
    LeftLanePos = distance(histogramLane.begin(), LeftPtr);
    
    vector<int>:: iterator RightPtr;
    RightPtr = max_element(histogramLane.begin()+250, histogramLane.end());
    RightLanePos = distance(histogramLane.begin(), RightPtr);
    
    line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0,255,0), 2);
    line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2);
    
   
    
}
void LaneCenter()
{
laneCenter = (RightLanePos-LeftLanePos)/2  +LeftLanePos;
frameCenter = 178; 

line(frameFinal, Point2f(laneCenter, 0), Point2f(laneCenter, 240), Scalar(0,255,0), 3);
line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255,0,0), 3);

 offsett = laneCenter-frameCenter;

}

void  Capture()
{
  Camera.grab();
  Camera.retrieve(frame);  
  cvtColor(frame, frame_Stop, COLOR_BGR2RGB);
  cvtColor(frame, frame_Traffic, COLOR_BGR2RGB);
  cvtColor(frame, frame_Pedestrian, COLOR_BGR2RGB);
  cvtColor(frame, frame, COLOR_BGR2RGB);
  
}


//Machine Learning functions

void Stop_detection(){
    if(!Stop_Cascade.load("/home/pi/Desktop/MachineLearning//Stop_cascade.xml"))
    { 
        cout<<"unable to open the cascade fil"<<endl;
    }
    ROI_Stop = frame_Stop(Rect(200,0,200,240));
    cvtColor(ROI_Stop, Gray_Stop, COLOR_RGB2GRAY);
    equalizeHist(Gray_Stop, Gray_Stop);
    Stop_Cascade.detectMultiScale(Gray_Stop,Stop);
    
    for(int i=0;i<  Stop.size() ;i++)
    {
        Point P1(Stop[i].x, Stop[i].y);
        Point P2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height);
        
        rectangle(ROI_Stop, P1, P2, Scalar(192,192,192), 1);
        putText(ROI_Stop, "Stop Sign", P1, FONT_HERSHEY_COMPLEX_SMALL, 0.6, Scalar(192,192,192), 1);
        
    distance_stop = (-0.9523)*(P2.x-P1.x)+107.2317;
        
    
    ss.str(" ");
    ss.clear();
    //ss<<"Distance  = "<<P2.x-P1.x<<"(Pixels)";
    ss<<"Distance = "<<distance_stop<<"cm";
    putText(ROI_Stop,ss.str(),Point2f(1,230), 0,0.55 ,Scalar(192,192,192),1.6);
    
    }
}

//traffic light
void Traffic_detection(){
    if(!Traffic_Cascade.load("/home/pi/Desktop/MachineLearning//Traffic_cascade.xml"))
    { 
        cout<<"unable to open the cascade Traffic fil"<<endl;
    }
    ROI_Traffic = frame_Traffic(Rect(200,0,200,240));
    cvtColor(ROI_Traffic, Gray_Traffic, COLOR_RGB2GRAY);
    equalizeHist(Gray_Traffic, Gray_Traffic);
    Traffic_Cascade.detectMultiScale(Gray_Traffic, Traffic);
    
    for(int i=0;i<  Traffic.size() ;i++)
    {
        Point P1(Traffic[i].x, Traffic[i].y);
        Point P2(Traffic[i].x + Traffic[i].width, Traffic[i].y + Traffic[i].height);
        
        rectangle(ROI_Traffic, P1, P2, Scalar(255,0,0), 1.5);
        putText(ROI_Traffic, "Red light", P1, FONT_HERSHEY_COMPLEX_SMALL, 0.6, Scalar(255,0,0), 1);
        
    distance_Traffic = (-2.2)*(P2.x-P1.x)+103.4;
            
    ss.str(" ");
    ss.clear();
    //ss<<"Distance  = "<<P2.x-P1.x<<"(Pixels)";
    ss<<"Distance = "<<distance_Traffic<<"cm";
    putText(ROI_Traffic,ss.str(),Point2f(1,230), 0,0.6 ,Scalar(255,0,0),1.6);
    
    }
}


//pedstrian

void Pedestrian_detection(){
    if(!Pedestrian_Cascade.load("/home/pi/Desktop/MachineLearning//Pedestrian.xml"))
    { 
        cout<<"unable to open the Pedestrian fil"<<endl;
    }
    ROI_Pedestrian = frame_Pedestrian;
    cvtColor(ROI_Pedestrian, Gray_Pedestrian, COLOR_RGB2GRAY);
    equalizeHist(Gray_Pedestrian, Gray_Pedestrian);
    Pedestrian_Cascade.detectMultiScale(Gray_Pedestrian, Pedestrian);
    
    for(int i=0;i<  Pedestrian.size() ;i++)
    {
        Point P1(Pedestrian[i].x, Pedestrian[i].y);
        Point P2(Pedestrian[i].x + Pedestrian[i].width, Pedestrian[i].y + Pedestrian[i].height);
        
        rectangle(ROI_Pedestrian, P1, P2, Scalar(255,0,0), 1.5);
        putText(ROI_Pedestrian, "Pedestrian", P1, FONT_HERSHEY_COMPLEX_SMALL, 0.6, Scalar(255,0,0), 1);
        
    distance_Pedestrian = (-1.6875)*(P2.x-P1.x)+165.125;
        
    
    ss.str(" ");
    ss.clear();
    //ss<<"Distance  = "<<P2.x-P1.x<<"(Pixels)";
    ss<<"Distance Pd = "<<distance_Pedestrian<<"cm";
    putText(ROI_Pedestrian,ss.str(),Point2f(1,230), 0,0.6 ,Scalar(255,0,0),1.6);
    
    }
}

//
class Sonar
{
  public:
    Sonar();
    void init(int trigger, int echo);
    double distance(int timeout);

  private:
    void recordPulseLength();
    int trigger;
    int echo;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;
    long travelTimeUsec;
    long now;
};

Sonar::Sonar(){}

void Sonar::init(int trigger, int echo)
{
    this->trigger=trigger;
    this->echo=echo;
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trigger, LOW);
    delay(500);
}

double Sonar::distance(int timeout)
{
    delay(10);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    now=micros();

    while (digitalRead(echo) == LOW && micros()-now<timeout);
        recordPulseLength();

    travelTimeUsec = endTimeUsec - startTimeUsec;
    distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;

    return distanceMeters;
}

void Sonar::recordPulseLength()
{
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH );
    endTimeUsec = micros();
}






int main(int argc,char **argv )
{
    wiringPiSetup();
    pinMode(21,OUTPUT);
    pinMode(22,OUTPUT);
    pinMode(23,OUTPUT);
    pinMode(24,OUTPUT);
    
    //ultrasonic setup_____________________________________________
   
      if (wiringPiSetup() == -1)
        return -1;

    Sonar sonar;
    sonar.init(TRIG, ECHO);
	float m=0;
    
    
    Setup(argc, argv, Camera);
    cout<<"Connecting to Camera"<<endl;
    if(!Camera.open())
    { 
        cout<<"Failed to connect"<<endl;
        return -1;
    }
    cout<< "camera ID = "<<Camera.getId()<<endl;
    
    while(1)
    {
    //____
   
    
	
    //_______
    auto start = std::chrono::system_clock::now();
    Capture();
    Perspective();
    threshold();
    histogram();
    LaneFinder();
    LaneCenter();
      
    
    //ML
    Stop_detection();
    Traffic_detection();
    Pedestrian_detection();
    m=sonar.distance(300000); 
    cout << "Distance is " <<m<< " cm." << endl;
    delay(100);
    if(m<20.0)
    {   
        digitalWrite(21,0);
        digitalWrite(22,1);  // décimale=10
        digitalWrite(23,0);
        digitalWrite(24,1);
        cout<<"Stop Obstacle"<<endl;
        goto stop_sign;
        
    }
     
    else if(distance_stop > 5 && distance_stop < 35)
    {
        digitalWrite(21,1);
        digitalWrite(22,1);// décimale=7
        digitalWrite(23,1);
        digitalWrite(24,0);
        cout<<"Stop Sign"<<endl;
        distance_stop=0;
        goto stop_sign;
    }
    else  if(distance_Traffic > 5 && distance_Traffic < 35)
    {
        digitalWrite(21,0);
        digitalWrite(22,0);// décimale=8
        digitalWrite(23,0);
        digitalWrite(24,1);
        cout<<"Red light"<<endl;
        distance_Traffic=0;
        goto traffic_detect;
    }
    else  if(distance_Pedestrian > 5 && distance_stop < 23)
    {
        digitalWrite(21,1);
        digitalWrite(22,0);// décimale=9
        digitalWrite(23,0);
        digitalWrite(24,1);
        cout<<"Pedestrian"<<endl;
        distance_stop=0;
        goto Pedestrian_detect;
    }
    else if(offsett >-4 && offsett<4 )
    {
        digitalWrite(21,0);
        digitalWrite(22,0);  // décimale=0
        digitalWrite(23,0);
        digitalWrite(24,0);
        cout<<"forward"<<endl;
        goto Pedestrian_detect;
    }
    else if(offsett>=4 && offsett <20)
    {
        digitalWrite(21,1);
        digitalWrite(22,0);
        digitalWrite(23,0);//  décimale=1
        digitalWrite(24,0);
        cout<<"right1"<<endl;
        goto stop_sign;
    }
    else if(offsett >= 20 && offsett<40)
    {
        digitalWrite(21,0);
        digitalWrite(22,1);
        digitalWrite(23,0);// décimale = 2
        digitalWrite(24,0);
        cout<<"Right2"<<endl;
        goto stop_sign;
    }
    else if(offsett >=40)
    {
        digitalWrite(21,1);
        digitalWrite(22,1); // décimale=3
        digitalWrite(23,0);
        digitalWrite(24,0);
        cout<<"right3"<<endl;
        goto stop_sign;
    }
    else if(offsett<=-4 && offsett > -20)
    {
        digitalWrite(21,0);
        digitalWrite(22,0);// décimale=4
        digitalWrite(23,1);
        digitalWrite(24,0);
        cout<<"left1"<<endl;
        goto stop_sign;
    }
    else if(offsett<=-20 && offsett > -40)
    {
        digitalWrite(21,1);
        digitalWrite(22,0);// décimale=5
        digitalWrite(23,1);
        digitalWrite(24,0);
        cout<<"left2"<<endl;
        goto stop_sign;
    }
    else if(offsett <= -40)
    {
        digitalWrite(21,0);
        digitalWrite(22,1);// décimale=6
        digitalWrite(23,1);
        digitalWrite(24,0);
        cout<<"left3"<<endl;
        goto stop_sign;
    }
    
    
    stop_sign:
    Pedestrian_detect:
    traffic_detect:
    
    
    ss.str(" ");
    ss.clear();
    ss<<"offset = "<<offsett;
    putText(frame,ss.str(),Point2f(1,50), 0,1 ,Scalar(255,0,0),2);
     
    cvtColor(frame, frame, COLOR_BGR2RGB);
    namedWindow("Original",WINDOW_KEEPRATIO);
    moveWindow("Original",0,100);
    resizeWindow("Original",640,480);
    imshow("Original",frame);
    
    namedWindow("Perspective",WINDOW_KEEPRATIO);
    moveWindow("Perspective",640,100);
    resizeWindow("Perspective",640,480);
    imshow("Perspective",framePerspect);
    
    namedWindow("Final",WINDOW_KEEPRATIO);
    moveWindow("Final",1280,100);
    resizeWindow("Final",640,480);
    imshow("Final",frameFinal);
   

   //namedWindow("Stop sign",WINDOW_KEEPRATIO);
   //moveWindow("Stop sign",0,580);
   //resizeWindow("Stop sign",640,480);
   //imshow("Stop sign",ROI_Stop);
   
    //cvtColor(ROI_Traffic, ROI_Traffic, COLOR_BGR2RGB);
    //namedWindow("Traffic_light",WINDOW_KEEPRATIO);
    //moveWindow("Traffic_light",640,580);
    //resizeWindow("Traffic_light",640,480);
    //imshow("Traffic_light",ROI_Traffic);
    
    cvtColor(ROI_Pedestrian, ROI_Pedestrian, COLOR_BGR2RGB);
    namedWindow("Pedestrian",WINDOW_KEEPRATIO);
    moveWindow("Pedestrian",1280,580);
    resizeWindow("Pedestrian",640,480);
    imshow("Pedestrian",ROI_Pedestrian);
    
  
        
    
    waitKey(1); 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    
    float t = elapsed_seconds.count();
    int FPS = 1/t;
    cout<<"FPS = "<<FPS<<endl;
    
    
    
    
    }

    return 0;
   
    
}
