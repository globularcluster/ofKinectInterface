#pragma once

#include "ofMain.h"

#include "ofxKCore.h"

#define MAIN_WINDOW_WIDTH 320.0f
#define MAIN_WINDOW_HEIGHT 280.0f

class ofApp : public ofBaseApp {
  public:
    void setup();
    void update();
    void draw();
    void exit();

    //  Load/save settings
    //
    bool loadXMLSettings();
    bool saveXMLSettings();

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y);
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

    void _ksetup();

    ofTrueTypeFont verdana;

    ofxKinect kinect;

    // Images
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage;       // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear;  // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar;   // the far thresholded image
    
    ofxCvGrayscaleImage sourceImg;
    CPUImageFilter processedImg;         // image after filters
    Filters* filter = NULL;

    //  Blob Tracker
    BlobTracker tracker;

    // Contour Finder
    ContourFinder contourFinder;
    float hullPress;


    // Calibration
    Calibration calib;

    //  XML Settings Vars
    //
    string message;

    bool bThreshWithOpenCV;
    bool bDrawPointCloud;

    // Kinect Camera
    int nearThreshold;
    int farThreshold;
    int camWidth;
    int camHeight;

    int maxBlobs;
    int MIN_BLOB_SIZE;
    int MAX_BLOB_SIZE;

    int angle;

    bool bDrawOutlines;
    bool bShowLabels;
};
