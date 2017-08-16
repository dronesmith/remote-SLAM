#include <stdio.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <KudanSLAM.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


std::vector<std::string> loadFolderContents(std::string folderName, bool doSort = true)
{


    std::vector<std::string> fileNames;

    //coutfn << "folderName is " << folderName << endl;
    if (folderName.size() == 0) {
        return fileNames;
    }


    DIR *dirHandle = opendir(folderName.c_str());

    if (dirHandle == NULL) {
        return fileNames;
    }

    dirent *fileEntry;
    std::string fileName;

    while ((fileEntry = readdir(dirHandle))) {
        fileName = fileEntry->d_name;

        if ((fileName == ".") || (fileName == "..") || (fileName.size() == 0) || (fileName[0] == '.')) {
            continue;
        }

        std::string filePath = folderName + "/" + fileName;
        fileNames.push_back(filePath);


    }

    closedir(dirHandle);

    if (doSort) {
        std::sort(fileNames.begin(), fileNames.end());
    }

    return fileNames;

}


int main(int argc, char **argv)
{
    printf("**This is a very simple demo of KudanSLAM (%s)! Should run on Mac and Linux** \n\n", KudanSLAM::version().c_str());



    if (argc <= 1) {
        printf("No input! Usage: ./VideoDemo inputFolder \n\n ");
        return 1;
    }

    std::string inputFolder = argv[1];

    std::vector<std::string> leftInputFiles;
    std::vector<std::string> rightInputFiles;

    std::string folderPathLeft = inputFolder + "/LeftCam";
    std::string folderPathRight = inputFolder + "/RightCam";

    leftInputFiles = loadFolderContents(folderPathLeft, true);
    rightInputFiles = loadFolderContents(folderPathRight, true);

    printf("Loaded %lu left files and %lu right files \n", leftInputFiles.size(), rightInputFiles.size());


    if (leftInputFiles.size() != rightInputFiles.size()) {
        printf("ERROR: Left right mismatch! \n");
        return 1;
    }

    size_t numFrames = leftInputFiles.size();

    if (numFrames == 0) {
        printf("ERROR: No input \n");
        return 1;
    }



    KudanSLAM slam;

    slam.setDescriptorMode();
    slam.setStereoMode();

    slam.setRequestPoints3D(true);
    slam.setRequestPoints2D(true);

    cv::Mat leftImage0 = cv::imread(leftInputFiles[0]);

    int calibrationWidth =  leftImage0.size().width;
    int calibrationHeight = leftImage0.size().height;

    printf("Width: %i", calibrationWidth);
    printf("Height: %i", calibrationHeight);

    if (calibrationWidth == 0 || calibrationHeight == 0) {
        printf("ERROR: Empty (first) image! \n");
        return 1;
    }

    // Hardcode for BLENDER
    slam.setCameraCalibration(calibrationWidth, calibrationHeight, 554.872, 553.433, 320, 180);


    KudanMatrix3 K = slam.getCalibrationMatrix();

    bool slamIsRunning = true;

    cv::namedWindow("SLAM Input");

    for (size_t v = 0; v < numFrames; v++) {

        cv::Mat leftImage = cv::imread(leftInputFiles[v]);
        cv::Mat rightImage = cv::imread(rightInputFiles[v]);

        if (leftImage.channels() == 3) {
            cv::cvtColor(leftImage, leftImage, CV_BGR2GRAY);
        }

        if (rightImage.channels() == 3) {
            cv::cvtColor(rightImage, rightImage, CV_BGR2GRAY);
        }

        // Save in a frame, for passing to SLAM
        KudanFrame currentFrame;
        currentFrame.leftImage.data = leftImage.data;
        currentFrame.leftImage.width = leftImage.size().width;
        currentFrame.leftImage.height = leftImage.size().height;

        currentFrame.rightImage.data = rightImage.data;
        currentFrame.rightImage.width = rightImage.size().width;
        currentFrame.rightImage.height = rightImage.size().height;



        if (slamIsRunning) {
            slam.processFrame(currentFrame);
        }

        // Initialise at the start
        if (v == 0) {
            slam.startTracking();
        }


        // DRAWING

        cv::Mat slamDisplay;
        cv::cvtColor(leftImage, slamDisplay, CV_GRAY2BGR);

        cv::Scalar keyframeColour(90, 100, 200);
        cv::Scalar projectedColour(240, 0, 100);
        cv::Scalar trackedColour(0, 0, 230);

        // Get the 2D points via the API
        std::vector<KudanVector2> projectedPoints = slam.getMapPoints2D();
        std::vector<KudanVector2> trackedPoints = slam.getTrackedPoints2D();


        for (KudanVector2 &point : projectedPoints) {
            cv::circle(slamDisplay, cv::Point2f(point.x, point.y), 3, projectedColour, 1);
        }


        for (KudanVector2 &point : trackedPoints) {
            cv::circle(slamDisplay, cv::Point2f(point.x, point.y), 5, trackedColour, 2);
        }

        // Text information

        float textWeight = 2;
        float textSize = 0.75;
        int textGap = 20;
        int textY = 20;

        if (slamIsRunning) {
            cv::putText(slamDisplay, "Slam Running", cv::Point(20, textY), 0, textSize, cv::Scalar(0, 250, 0), textWeight);
            textY += textGap;
        }

        cv::putText(slamDisplay, "Keyframes: " + std::to_string(slam.getNumKeyframes()), cv::Point(20, textY), 0, textSize, keyframeColour, textWeight);
        textY += textGap;
        cv::putText(slamDisplay, "Points: " + std::to_string(projectedPoints.size()), cv::Point(20, textY), 0, textSize, projectedColour, textWeight);
        textY += textGap;
        cv::putText(slamDisplay, "Tracked: " + std::to_string(trackedPoints.size()), cv::Point(20, textY), 0, textSize, trackedColour, textWeight);
        textY += textGap;


        cv::imshow("SLAM Input", slamDisplay);

        // Very simple UI (keys, managed by OpenCV)
        bool stopLooping = false;
        int keyId = cv::waitKey(1);

        switch (keyId) {
            case 27:
                stopLooping = true;
                break;

            case -1:
                break;

            default:
                printf("Unknown key ID = %i \n", keyId);
        }

        if (stopLooping) {
            break;
        }
    }

    printf("byee\n");
}
