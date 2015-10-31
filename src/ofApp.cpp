#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	//  ofSetLogLevel(OF_LOG_VERBOSE);

	nearThreshold = 255;
	farThreshold = 230;

	// initialize Kinect
	_ksetup();

	camWidth = kinect.width;
	camHeight = kinect.height;
	srcPoints[0] = dstPoints[0] = ofPoint(0, 0);
	srcPoints[1] = dstPoints[1] = ofPoint(camWidth, 0);
	srcPoints[2] = dstPoints[2] = ofPoint(camWidth, camHeight);
	srcPoints[3] = dstPoints[3] = ofPoint(0, camHeight);

	// create filter
	filter = new ProcessFilters();

	// load settings from config.xml
	loadXMLSettings();

	ofSetVerticalSync(false);  // Set vertical sync to false
							   // for better performance?

	// Allocate images
	//
	colorImg.allocate(camWidth, camHeight);
	grayImage.allocate(camWidth, camHeight);
	//grayImage.setUseTexture(false);
	grayThreshNear.allocate(camWidth, camHeight);
	grayThreshFar.allocate(camWidth, camHeight);

	processedImg.allocate(camWidth, camHeight);
	//processedImg.setUseTexture(false);

	// setup calibration
	calib.setup(camWidth, camHeight, &tracker);

	// allocate filters
	filter->allocate(camWidth, camHeight);

	verdana.loadFont("verdana.ttf", 8, true, true);

	bThreshWithOpenCV = true;

	ofSetFrameRate(30);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	//GUI
	testeGroup.setup();
	parametersGroup.add(testeGroup.guiParameters);
	parametersGroup.add(testeGroup.outrosParameters);
	gui.setup(parametersGroup);
}

//--------------------------------------------------------------
void ofApp::update() {
	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width,
				kinect.height);

		// we do two thresholds - one for the far plane and one for the near
		// plane
		// we then do a cvAnd to get the pixels which are a union of the two
		// thresholds
		if (bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(),
					grayImage.getCvImage(), NULL);
		} else {
			// or we do it ourselves - show people how they can work with the
			// pixels
			unsigned char* pix = grayImage.getPixels();

			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for (int i = 0; i < numPixels; i++) {
				if (pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}

		// update the cv images
		grayImage.flagImageChanged();

		processedImg = grayImage;

		// DO IT!
		filter->applyFilters(processedImg, srcPoints, dstPoints);

		contourFinder.findContours(processedImg, (MIN_BLOB_SIZE * 2) + 1,
				((camWidth * camHeight) * .4) * (MAX_BLOB_SIZE * .001), 10, 20,
				false);

		if (contourFinder.bTrackBlobs) {
			for (int i = 0; i < contourFinder.blobs.size(); i++) {
				ofPoint pos = contourFinder.blobs[i].centroid;
				contourFinder.blobs[i].centroid.y = kinect.getDistanceAt(pos);
			}
		}

		if (contourFinder.bTrackFingers) {
			for (int i = 0; i < contourFinder.fingers.size(); i++) {
				ofPoint pos = contourFinder.fingers[i].centroid;
				contourFinder.fingers[i].centroid.y = kinect.getDistanceAt(pos);
			}
		}

		tracker.track(&contourFinder);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255, 255, 255);

	// draw from the live kinect
	// kinect.drawDepth(10, 10, 400, 300);
	kinect.draw(10, 10, MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT);

	processedImg.draw(MAIN_WINDOW_WIDTH + 20, 10, MAIN_WINDOW_WIDTH,
	MAIN_WINDOW_HEIGHT);

	if (contourFinder.bTrackBlobs) {
		for (int i = 0; i < contourFinder.nBlobs; i++) {
			if (bDrawOutlines)  // Draw contours (outlines) on the source
								// image
				contourFinder.blobs[i].drawContours(
				MAIN_WINDOW_WIDTH + 20, 10, camWidth, camHeight,
				MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT);

			if (bShowLabels)  // Show ID label
			{
				float xpos = contourFinder.blobs[i].centroid.x
						* (MAIN_WINDOW_WIDTH / camWidth);
				float ypos = contourFinder.blobs[i].centroid.y
						* (MAIN_WINDOW_HEIGHT / camHeight);

				ofSetColor(200, 255, 200);
				char idStr[1024];

				sprintf(idStr, "id: %i", contourFinder.blobs[i].id);
				verdana.drawString(idStr, xpos + MAIN_WINDOW_WIDTH + 20,
						ypos + contourFinder.blobs[i].boundingRect.height / 2
								+ 10);
			}
		}
	}

	//  Find the blobs for drawing
	//
	if (contourFinder.bTrackFingers) {
		for (int i = 0; i < contourFinder.nFingers; i++) {
			if (bDrawOutlines)  // Draw contours (outlines) on the source
								// image
				contourFinder.fingers[i].drawCenter(
				MAIN_WINDOW_WIDTH + 20, 10, camWidth, camHeight,
				MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT);

			if (bShowLabels)  // Show ID label
			{
				float xpos = contourFinder.fingers[i].centroid.x
						* (MAIN_WINDOW_WIDTH / camWidth);
				float ypos = contourFinder.fingers[i].centroid.y
						* (MAIN_WINDOW_HEIGHT / camHeight);

				stringstream testeStream;
				testeStream << "finger: " << i << "\nxpos: " << (int) xpos
						<< "\typos: " << (int) ypos << endl;

				ofDrawBitmapString(testeStream.str(), 10,
				MAIN_WINDOW_WIDTH + (i * 30));

				ofSetColor(200, 255, 200);
				char idStr[1024];

				sprintf(idStr, "id: %i", contourFinder.fingers[i].id);

				verdana.drawString(idStr, xpos + MAIN_WINDOW_WIDTH + 20,
						ypos + contourFinder.fingers[i].boundingRect.height / 2
								+ 10);
			}
		}
	}

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;

	reportStream << "using opencv threshold = " << bThreshWithOpenCV
			<< " (press spacebar)" << endl << "set near threshold "
			<< nearThreshold << " (press: + -)" << endl << "set far threshold "
			<< farThreshold << " (press: < >) num blobs found "
			<< contourFinder.nBlobs << ", fps: " << ofGetFrameRate() << endl
			<< "press c to close the connection and o to open it again, "
					"connection is: " << kinect.isConnected() << endl;

	if (kinect.hasCamTiltControl()) {
		reportStream << "press UP and DOWN to change the tilt angle: " << angle
				<< " degrees" << endl << "press 1-5 & 0 to change the led mode"
				<< endl;
	}

	// ofDrawBitmapString(reportStream.str(), 20, 360);

	gui.draw();

}

void ofApp::exit() {
	kinect.setCameraTiltAngle(0);  // zero the tilt on exit
	delete filter;
	filter = NULL;
	kinect.close();
}

bool ofApp::loadXMLSettings() {
	message = "Loading config.xml...";

	ofxXmlSettings XML;
	if (XML.loadFile("config.xml")) {
		if (XML.pushTag("CONFIG")) {
			nearThreshold = XML.getValue("KINECT:NEAR", 255);
			farThreshold = XML.getValue("KINECT:FAR", 230);

			maxBlobs = XML.getValue("BLOBS:MAXNUMBER", 20);

			bShowLabels = XML.getValue("BOOLEAN:LABELS", 1);
			bDrawOutlines = XML.getValue("BOOLEAN:OUTLINES", 1);

			//  Pre-Filter
			//
			filter->bLearnBakground = XML.getValue("BOOLEAN:LEARNBG", 0);
			filter->bVerticalMirror = XML.getValue("BOOLEAN:VMIRROR", 0);
			filter->bHorizontalMirror = XML.getValue("BOOLEAN:HMIRROR", 0);

			//  Filters
			//
			filter->bTrackDark = XML.getValue("BOOLEAN:TRACKDARK", 0);
			filter->bHighpass = XML.getValue("BOOLEAN:HIGHPASS", 0);
			filter->bAmplify = XML.getValue("BOOLEAN:AMPLIFY", 0);
			filter->bSmooth = XML.getValue("BOOLEAN:SMOOTH", 0);
			filter->bDynamicBG = XML.getValue("BOOLEAN:DYNAMICBG", 0);

			//  Filter Settings
			//
			filter->threshold = XML.getValue("INT:THRESHOLD", 0);
			filter->highpassBlur = XML.getValue("INT:HIGHPASSBLUR", 0);
			filter->highpassNoise = XML.getValue("INT:HIGHPASSNOISE", 0);
			filter->highpassAmp = XML.getValue("INT:HIGHPASSAMP", 0);
			filter->smooth = XML.getValue("INT:SMOOTH", 0);

			//  CounterFinder
			//
			MIN_BLOB_SIZE = XML.getValue("INT:MINBLOBSIZE", 2);
			MAX_BLOB_SIZE = XML.getValue("INT:MAXBLOBSIZE", 100);
			hullPress = XML.getValue("INT:HULLPRESS", 20.0);
			tracker.MOVEMENT_FILTERING = XML.getValue("INT:MINMOVEMENT", 0);

			//  Tracking Options
			//
			contourFinder.bTrackBlobs = XML.getValue("BOOLEAN:TRACKBLOBS", 0);
			contourFinder.bTrackFingers = XML.getValue("BOOLEAN:TRACKFINGERS",
					0);
			contourFinder.bTrackObjects = XML.getValue("BOOLEAN:TRACKOBJECTS",
					0);

			//  NETWORK SETTINGS
			//
			// bTUIOMode = XML.getValue("BOOLEAN:TUIO", 0);
			// myTUIO.bOSCMode = XML.getValue("BOOLEAN:OSCMODE", 1);
			// myTUIO.bTCPMode = XML.getValue("BOOLEAN:TCPMODE", 1);
			// myTUIO.bBinaryMode = XML.getValue("BOOLEAN:BINMODE", 1);
			// myTUIO.bHeightWidth = XML.getValue("BOOLEAN:HEIGHTWIDTH", 0);
			// tmpLocalHost = XML.getValue("NETWORK:LOCALHOST", "localhost");
			// tmpPort = XML.getValue("NETWORK:TUIOPORT_OUT", 3333);
			// tmpFlashPort = XML.getValue("NETWORK:TUIOFLASHPORT_OUT", 3000);

			// myTUIO.setup(tmpLocalHost.c_str(), tmpPort,
			//              tmpFlashPort);  // have to convert tmpLocalHost to a
			//                              // const char*

			XML.popTag();

			message = "Settings Loaded!\n\n";
			return true;
		} else {
			message = "The settings file was empty!\n\n";
			return false;
		}
	} else {
		message = "No Settings Found...\n\n";  // FAIL
		return false;
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case ' ':
		bThreshWithOpenCV = !bThreshWithOpenCV;
		break;

	case 'd':
		bDrawOutlines = !bDrawOutlines;
		bShowLabels = !bShowLabels;
		break;

	case '>':
	case '.':
		farThreshold++;
		if (farThreshold > 255)
			farThreshold = 255;
		break;

	case '<':
	case ',':
		farThreshold--;
		if (farThreshold < 0)
			farThreshold = 0;
		break;

	case '+':
	case '=':
		nearThreshold++;
		if (nearThreshold > 255)
			nearThreshold = 255;
		break;

	case '-':
		nearThreshold--;
		if (nearThreshold < 0)
			nearThreshold = 0;
		break;

	case 'w':
		kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		break;

	case 'o':
		kinect.setCameraTiltAngle(angle);  // go back to prev tilt
		kinect.open();
		break;

	case 'c':
		kinect.setCameraTiltAngle(0);  // zero the tilt
		kinect.close();
		break;
	case 'q':
		ofExit();
		break;

	case '1':
		kinect.setLed(ofxKinect::LED_GREEN);
		break;

	case '2':
		kinect.setLed(ofxKinect::LED_YELLOW);
		break;

	case '3':
		kinect.setLed(ofxKinect::LED_RED);
		break;

	case '4':
		kinect.setLed(ofxKinect::LED_BLINK_GREEN);
		break;

	case '5':
		kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
		break;

	case '0':
		kinect.setLed(ofxKinect::LED_OFF);
		break;

	case OF_KEY_UP:
		angle++;
		if (angle > 30)
			angle = 30;
		kinect.setCameraTiltAngle(angle);
		break;

	case OF_KEY_DOWN:
		angle--;
		if (angle < -30)
			angle = -30;
		kinect.setCameraTiltAngle(angle);
		break;
	}
}

void ofApp::_ksetup() {
	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	// kinect.init(true); // shows infrared instead of RGB video image
	// kinect.init(false, false); // disable video image (faster fps)

	kinect.open();  // opens first available kinect

	if (kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: "
				<< kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  "
				<< kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: "
				<< kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance()
				<< "mm";
	} else {
		ofLogNotice() << "Kinect is not conected!";
		exit();
	}

	kinect.setDepthClipping();
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {
}
