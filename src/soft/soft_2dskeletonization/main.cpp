/*
Copyright (c) 2016 Bastien Durix

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/**
 *  \brief 2D skeletonization
 *  \author Bastien Durix
 */

#include <boost/program_options.hpp>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <fstream>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <filesystem>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <shape/DiscreteShape.h>
#include <boundary/DiscreteBoundary.h>
#include <skeleton/Skeletons.h>

#include <algorithm/extractboundary/NaiveBoundary.h>
#include <algorithm/skeletonization/SpherePropagation2D.h>
#include <algorithm/skinning/Filling.h>
#include <algorithm/evaluation/ReprojError.h>

#include <displayopencv/DisplayShapeOCV.h>
#include <displayopencv/DisplayBoundaryOCV.h>
#include <displayopencv/DisplaySkeletonOCV.h>

using namespace std;
using namespace cv;

//Declaration default values
string inputImgDefault = "RK5_20200104_SHSY5Y_R_2500_03_Alexa488_02.png";
string skeletonImgNameDefault = "skeleton.png";
string filenameEnding = "-Epsilon1px-skeleton.png";
double epsilonValueDefault = 10.0;
bool outputDefault = true;
bool variableOutputNamesDefault = true;
bool openDirectory = true;


//Declaration globale values
std::string imgfile, skeletonImgName, prefix, resultFilename;
bool output = false;
double epsilon;
bool variableOutputNames;

/**
 *
 * @param str src Alexa file
 * @return Name for associated DAPI file
 */
string replaceSubstring(string str);

/**
 *
 * @param directoryName Name of the directory to grab at
 * @return Error numbers for failure
 */
int inputFolderGrabbing(const char *directoryName);

/**
 *
 * @return
 */
Mat simpleRead();

/**
 * Reads the command line parameters and input it to globale params
 * @param argc System variable
 * @param argv System variable
 */
int inputValuesRead(int argc, char **argv);

/**
 * Generates variable file names
 * @param filenameSuffix the suffix for the filename
 * @return the generated filename
 */
string setVariableFilenames(string filenameSuffix, int i);

/**
 *
 * @param dissh
 * @param disbnd
 * @param skel
 * @return
 */
tuple<double, double, int, int> EvalSkel(const shape::DiscreteShape<2>::Ptr dissh,
                                         const boundary::DiscreteBoundary<2>::Ptr disbnd,
                                         const skeleton::GraphSkel2d::Ptr skel);

/**
 * Print the Skeleton Counter for the hole Image
 * @param skeletonPointsCounter
 */
void consoleOutputCompleteData(int skeletonPointsCounter);

/**
 *
 * @param srcAlexa
 * @return
 */
Mat distanceTransformAlexa(Mat srcAlexa);

/**
 *
 * @param srcSkeleton
 * @param dist
 * @return
 */
Mat substractDistFromSkeletonfile(Mat srcSkeleton, Mat dist);

/**
 * Generates the complete data image with shape, boundary, boundary differece and skelett
 * @param image output image
 * @param dissh shape
 * @param disbnd boundary
 * @param shppropag shape propagation
 * @param grskelpropag skeleton
 * @return the matrix of the complete image
 */
Mat generateCompleteImage(Mat image, shape::DiscreteShape<2>::Ptr dissh, boundary::DiscreteBoundary<2>::Ptr disbnd,
                          shape::DiscreteShape<2>::Ptr shppropag, skeleton::GraphSkel2d::Ptr grskelpropag, int i);

/**
 * Generate the skeleton inputImage and returns it
 * @param inputImage input inputImage, origin inputImage for the program
 * @param dissh shape
 * @param grskelpropag skeleton
 * @return the matrix of the skeleton inputImage, zero = background and 255 = foreground, simple black and white inputImage
 */
Mat generateSkeletonImage(Mat inputImage, shape::DiscreteShape<2>::Ptr dissh, skeleton::GraphSkel2d::Ptr grskelpropag,
                          int i);

/**
 * Generate the boundary image and returns it
 * @param image input image, origin image for the program
 * @param dissh shape
 * @param disbnd boundary
 * @return the matrix of the boundary image, zero = background and 255 = foreground, simple black and white image
 */
Mat
generateBoundaryImage(Mat image, shape::DiscreteShape<2>::Ptr dissh, boundary::DiscreteBoundary<2>::Ptr disbnd, string filenameSuffix,
        int i);

/**
 *
 * @param srcAlexa
 * @param srcDapi
 */
void splitContours(Mat srcAlexa);

void writeCSVDataResult(list<int> nodeList, list<int> branchList, list<double> distanceList, list<int> timeList,
                        list<int> skeletonPointSingleCountList, int SkeletonPointsWithoutDistTrans, string filenameSuffix);

int main(int argc, char **argv) {
    //system("exec rm -r ../output/*");
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y");
    auto str = oss.str();
    prefix = str;
    system("../test.sh");
    system(" /opt/fiji/Fiji.app/ImageJ-linux64 -ij2 --headless --console -macro ../test2.ijm ../ressources/");
    resultFilename = "../output/"+ prefix + "/resultData.csv";
    inputValuesRead(argc, argv);

    if (variableOutputNames) {
        skeletonImgName = setVariableFilenames(filenameEnding, 0);
    }
    int test = inputFolderGrabbing("../ressources");
    cout << "fertig" <<endl;
    return test;
}

string replaceSubstring(string str){
    size_t index = 0;
    while (true) {
        /* Locate the substring to replace. */
        index = str.find("Alexa", index);
        if (index == std::string::npos) break;

        /* Make the replacement. */
        str.replace(index, 8, "DAPI");

        /* Advance index forward so the next iteration doesn't pick it up as well. */
        index += 3;
    }
    return str;
}

int inputFolderGrabbing(const char *directoryName){
    DIR *dir;
    struct dirent *ent;
    string dirName;
    if ( openDirectory == true) {
        if ((dir = opendir(directoryName)) != NULL) {
            while ((ent = readdir(dir))) {
                dirName = ent->d_name;
                int test = 1;
                if (dirName != "." && dirName != ".." && dirName != ".git") {
                    //picture data found
                    if (dirName.find(".png") != string::npos){
//                        cout << dirName << endl;
                        imgfile = directoryName;
                        imgfile.append("/" + dirName);
                        Mat outClosing = simpleRead();
                    }
                    else if (dirName.find(".tif") != string::npos)
                    {
                        //tue nichts
//                        cout << dirName << endl;
//                        cout << "tue nichts" << endl;
                    }
                    //directory found
                    else {
                        //cout << dirName << endl;
                        string test = directoryName;
                        test.append("/" + dirName);
                        int n = test.length();
                        char char_array[n+1];
                        strcpy (char_array, test.c_str());
                        const char *dirNeu = char_array;
                        inputFolderGrabbing(dirNeu);
                    }
                }
            }
            closedir(dir);
        } else {
            perror("");
            return EXIT_FAILURE;
        }
    }else {
        Mat outClosing = simpleRead();
    }
    //exit program
    return 0;
}

int inputValuesRead(int argc, char **argv) {
    boost::program_options::options_description desc("OPTIONS");
    desc.add_options()
            ("help", "Help message")
            ("imgfile",
             boost::program_options::value<std::string>(&imgfile)->default_value("../ressources/" + inputImgDefault),
             "Greyscale image file with file extension")
            ("output", boost::program_options::value<bool>(&output)->default_value(outputDefault),
             "Returns output images")
            ("epsilon", boost::program_options::value<double>(&epsilon)->default_value(epsilonValueDefault),
             "Skeleton precision")
            ("variableOutputNames", boost::program_options::value<bool>(&variableOutputNames)->
                     default_value(variableOutputNamesDefault),
             "Variable Outputnames allowed")
            ("skeletonImgName", boost::program_options::value<string>(&skeletonImgName)->
                    default_value(skeletonImgNameDefault), "Skeleton img file");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (epsilon != 1) {
        ostringstream strs;
        strs << epsilon;
        string str = strs.str();
        filenameEnding = "-Epsilon" + str + "px-skeleton.png";
    }

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    return 1;
}

string setVariableFilenames(string filenameSuffix, int i) {
    string first = imgfile.substr(14);
    int filenameLength = first.length();
    string filename = first.substr(0, filenameLength - 4);

    string generatedFilename;

    if (i == 0) {
        generatedFilename = "../output/" + prefix + "/" + filename + filenameSuffix;
    } else {
        stringstream ss;
        ss << i;
        string str = ss.str();
        filename = "../output/" + prefix + "/" + filename + "_" + str;
        generatedFilename = filename + filenameSuffix;
    }
    return generatedFilename;
}

cv::Mat simpleRead() {
    Mat matAlexaFile = imread(imgfile);
    if (matAlexaFile.empty()) {
        throw logic_error("Wrong input data Alexa file...");
    }
//    string dapiFile = replaceSubstring(imgfile);
//    Mat matDapiFile = imread(dapiFile);
//    if (matDapiFile.empty()) {
//        throw logic_error("Wrong input data DAPI file...");
//    }
    splitContours(matAlexaFile);
    return matAlexaFile;
}

tuple<double, double, int, int> EvalSkel(const shape::DiscreteShape<2>::Ptr dissh,
                                         const boundary::DiscreteBoundary<2>::Ptr disbnd,
                                         const skeleton::GraphSkel2d::Ptr skel) {
    shape::DiscreteShape<2>::Ptr shp(new shape::DiscreteShape<2>(dissh->getWidth(), dissh->getHeight()));
    algorithm::skinning::Filling(shp, skel);

    double res = algorithm::evaluation::SymDiffArea(dissh, shp);
    double res2 = algorithm::evaluation::HausDist(skel, disbnd, dissh->getFrame());

    list<unsigned int> lnod;
    skel->getAllNodes(lnod);
    unsigned int nbbr = 0;
    for (std::list<unsigned int>::iterator it = lnod.begin(); it != lnod.end(); it++) {
        unsigned int deg = skel->getNodeDegree(*it);
        if (deg != 2)
            nbbr += deg;
    }
    nbbr /= 2;
    tuple<double, double, int, int> result = std::make_tuple(res * 100.0, res2, skel->getNbNodes(), nbbr);

    return result;
}

void consoleOutputCompleteData(int skeletonPointsCounter) {
    cout << "Number of skeleton points complete image: " << skeletonPointsCounter << endl;
}

Mat generateCompleteImage(Mat image, shape::DiscreteShape<2>::Ptr dissh, boundary::DiscreteBoundary<2>::Ptr disbnd,
                          shape::DiscreteShape<2>::Ptr shppropag, skeleton::GraphSkel2d::Ptr grskelpropag, int i) {
    /*Mat imagepropag;
    image.copyTo(imagepropag);*/

    displayopencv::DisplayDiscreteShape(dissh, image, shppropag->getFrame(),
                                        cv::Scalar(255, 0, 0));
    displayopencv::DisplayDiscreteShape(shppropag, image, shppropag->getFrame(),
                                        cv::Scalar(125, 125, 125));
    displayopencv::DisplayDiscreteBoundary(disbnd, image, dissh->getFrame(),
                                           cv::Scalar(0, 0, 0));
    displayopencv::DisplayGraphSkeleton(grskelpropag, image, dissh->getFrame(),
                                        cv::Scalar(255, 0, 0));

    if (output && i == 0) {
        string filename = setVariableFilenames("-skeleton.png", i);
        imwrite(filename, image);
    }

    return image;
}

Mat generateSkeletonImage(Mat inputImage, shape::DiscreteShape<2>::Ptr dissh, skeleton::GraphSkel2d::Ptr grskelpropag,
                          int i) {
    displayopencv::DisplayGraphSkeleton(grskelpropag, inputImage, dissh->getFrame(),
                                        cv::Scalar(255, 255, 255));
    string filename = setVariableFilenames("-SkeletonImg.png", i);
    if (i == 0) {
        imwrite(filename, inputImage);
    }
    return inputImage;
}

//"-BoundaryImg.png"
Mat generateBoundaryImage(Mat image, shape::DiscreteShape<2>::Ptr dissh, boundary::DiscreteBoundary<2>::Ptr disbnd, string filenameSuffix,
        int i) {
    displayopencv::DisplayDiscreteBoundary(disbnd, image, dissh->getFrame(),
                                           cv::Scalar(255, 255, 255));
    string filename = setVariableFilenames(filenameSuffix, i);
    if (i == 0) {
        imwrite(filename, image);
    }
    return image;
}

vector<pair<int, int>> getAllImageCoordinates(Mat img) {
    vector<pair<int, int>> coordinateList;
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            uchar value = img.at<uchar>(y, x);
            if (value != 0) {
                coordinateList.push_back(make_pair(x, y));
            }
        }
    }
    return coordinateList;
}

Mat distanceTransformAlexa(Mat srcAlexa){
    Mat dist;
    distanceTransform(srcAlexa, dist, DIST_C, 3);
    normalize(dist, dist, 0, 1.0, NORM_MINMAX);
    threshold(dist, dist, 0.4, 255, THRESH_BINARY);
    Mat kernel1 = Mat::ones(3, 3, CV_8U);
    dilate(dist, dist, kernel1);
    dist.convertTo(dist, CV_8U);
    imwrite("DistanceTransform.png", dist);
    return dist;
}

Mat substractDistFromSkeletonfile(Mat srcSkeleton, Mat dist){
    Mat result;
    auto type1 = srcSkeleton.type();
    auto type2 = dist.type();
    cv::subtract(srcSkeleton, dist, result);
    imwrite("Substract.png", result);
    return result;
}

void splitContours(Mat srcAlexa) {
    Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
    Mat imgLaplacian;
    filter2D(srcAlexa, imgLaplacian, CV_32F, kernel);
    Mat sharp;
    srcAlexa.convertTo(sharp, CV_32F);
    Mat imgResult = sharp - imgLaplacian;

    //convert 8B to greyscale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);

    //imwrite("Laplacian_finltered_image.png", imgLaplacian);
    //imwrite("New_sharped_image.png", imgResult);

    //create binary image
    Mat bw;
    cvtColor(imgResult, bw, COLOR_BGR2GRAY);
    threshold(bw, bw, 40, 255, THRESH_BINARY | THRESH_OTSU);
//    Mat element = getStructuringElement(cv::MORPH_RECT,Size(3,3),Point(1,1));
//    morphologyEx(bw, bw, MORPH_CLOSE, element);

    //imwrite("Binary_image.png", bw);
    resize(bw, bw, Size(bw.cols * 3, bw.rows * 3));
    Mat element = getStructuringElement(cv::MORPH_RECT,Size(3,3),Point(1,1));
    morphologyEx(bw, bw, MORPH_CLOSE, element);
    imwrite("../output/Mopho_Output.png", bw);

//    Mat dist;
//    distanceTransform(bw, dist, DIST_L2, 3);
//    imwrite("../output/Distance.png", dist);


    // create CV_8U of distance image, needed for find conturs
    Mat dist_8u;
    bw.convertTo(dist_8u, CV_8U);

    Mat dist = distanceTransformAlexa(bw);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(dist_8u, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_L1);

    Mat test = Mat::zeros(dist_8u.size(), CV_8UC3);
    std::string str;

    //Only get the contours on the first layer (foreground contours)
    if (!contours.empty() && !hierarchy.empty()) {
        Mat completeContour = Mat::zeros(dist_8u.size(), CV_8UC3);
        Mat completeIMG(dist_8u.size(), CV_8UC3, Scalar(255, 255, 255));
        Mat completeBoundary = Mat::zeros(dist_8u.size(), CV_8UC3);
        Mat completeSkeleton = Mat::zeros(dist_8u.size(), CV_8UC3);

        list<int> nodeList;
        list<int> branchList;
        list<double> distanceList;
        list<int> skeletonPointSingleCountList;
        list<int> timeList;

        int indx = 1;
        cout << contours.size() << endl;
        for (int i = 0; i <= contours.size(); i++) {
            if (hierarchy[i][3] == -1) {
                double area = contourArea(contours[i]);
                if (!(area <=200)) {
                    Mat singleContour = Mat::zeros(dist_8u.size(), CV_8UC3);
                    Scalar color(rand() & 255, rand() & 255, rand() & 255);
                    drawContours(singleContour, contours, (int) i, color, FILLED, 8, hierarchy);
                    drawContours(completeContour, contours, (int) i, color, FILLED, 8, hierarchy);
                    imwrite("../output/SingleContour.png", singleContour);

                    threshold(singleContour, singleContour, 1, 255, THRESH_BINARY);
                    cvtColor(singleContour, singleContour, COLOR_BGR2GRAY);
                    imwrite("../output/Mopho_Output.png", singleContour);

                    shape::DiscreteShape<2>::Ptr dissh = shape::DiscreteShape<2>::Ptr(
                            new shape::DiscreteShape<2>(singleContour.cols,
                                                        singleContour.rows));
                    Mat cpymat(singleContour.rows, singleContour.cols, CV_8U, &dissh->getContainer()[0]);
                    singleContour.copyTo(cpymat);
                    Mat image(singleContour.rows, singleContour.cols, CV_8UC3, cv::Scalar(255, 255, 255));

                    boundary::DiscreteBoundary<2>::Ptr disbnd = algorithm::extractboundary::NaiveBoundary(dissh);

                    auto start0 = std::chrono::steady_clock::now();
                    algorithm::skeletonization::propagation::OptionsSphProp options(2.0 * epsilon);
                    map<pair<int, int>, vector<vector<pair<int, int>>>> contractlist;
                    skeleton::GraphSkel2d::Ptr grskelpropag = algorithm::skeletonization::propagation::SpherePropagation2D(
                            contractlist, disbnd,
                            options);
                    auto duration0 = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start0);
                    tuple<double, double, int, int> respropag = EvalSkel(dissh, disbnd, grskelpropag);
                    int t0 = duration0.count();


                    shape::DiscreteShape<2>::Ptr shppropag(
                            new shape::DiscreteShape<2>(dissh->getWidth(), dissh->getHeight()));
                    algorithm::skinning::Filling(shppropag, grskelpropag);

                    Mat skelImg = Mat::zeros(image.rows, image.cols, CV_8UC1);
                    Mat skeletonImg = generateSkeletonImage(skelImg, dissh, grskelpropag, indx);
                    generateSkeletonImage(completeSkeleton, dissh, grskelpropag, 0);
                    SparseMat newMat(skeletonImg);
                    int SkeletonPointsCounter = newMat.nzcount();
                    //writeCSVData(skelPointsList, "-skeletonData.csv", indx);
                    //consoleOutputSingleData(respropag, t0, SkeletonPointsCounter);
                    if (SkeletonPointsCounter != 0){
                        Mat imagepropag(dist_8u.size(), CV_8UC3, Scalar(255, 255, 255));
                        //generateCompleteImage(imagepropag, dissh, disbnd, shppropag, grskelpropag, indx);
                        generateCompleteImage(completeIMG, dissh, disbnd, shppropag, grskelpropag, 0);

                        Mat boundImg = Mat::zeros(image.rows, image.cols, CV_8UC1);
                        //Mat boundaryImg = generateBoundaryImage(boundImg, dissh, disbnd, indx);

                        generateBoundaryImage(completeBoundary, dissh, disbnd, "-BoundaryImg.png",  0);

                        nodeList.push_back(get<2>(respropag));
                        branchList.push_back(get<3>(respropag));
                        distanceList.push_back(get<1>(respropag));
                        timeList.push_back(t0);
                        skeletonPointSingleCountList.push_back(SkeletonPointsCounter);
                        indx++;

                    }
                }
            }
        }
        ifstream file(resultFilename);
        //check if file not exists and creates one with headlines
        if(!file.good()){
            ofstream csvFile(resultFilename);
            csvFile << "Dateiname ; Anzahl Nodes ; Anzahl Branches ; Hausdorff Distance (px) ; Berechnungszeit (ms) ; Skelettpunkte ; Skelettpunkte ohne DistanceTranform \n";
            csvFile.close();
        }
        SparseMat newMat(completeSkeleton);
        int SkeletonPointsCounterComplete = newMat.nzcount();
        cvtColor(completeSkeleton, completeSkeleton, COLOR_BGR2GRAY);
        threshold(completeSkeleton, completeSkeleton, 1, 255, THRESH_BINARY | THRESH_OTSU);
        Mat result = substractDistFromSkeletonfile(completeSkeleton, dist);
        SparseMat newMat2(result);
        int SkeletonPointsCounterComplete2 = newMat2.nzcount();
        consoleOutputCompleteData(SkeletonPointsCounterComplete);
        consoleOutputCompleteData(SkeletonPointsCounterComplete2);
        imwrite("completeContour.png", completeContour);
        writeCSVDataResult(nodeList, branchList, distanceList, timeList, skeletonPointSingleCountList, SkeletonPointsCounterComplete2,
                           resultFilename);
        Mat test;
        cv::subtract(bw, result, test);
        string filename = setVariableFilenames("-Complete.png", 0);
        imwrite(filename, test);
    } else {
        throw logic_error("No contours found...");
    }
    findContours(dist_8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}

void writeCSVDataResult(list<int> nodeList, list<int> branchList, list<double> distanceList, list<int> timeList,
                        list<int> skeletonPointSingleCountList, int skelPointsDistTrans, string filenameSuffix) {
    list<int>::iterator itNodes = nodeList.begin();
    list<int>::iterator itBranches = branchList.begin();
    list<double>::iterator itDistances = distanceList.begin();
    list<int>::iterator itTimes = timeList.begin();
    list<int>::iterator itSkeletonPoints = skeletonPointSingleCountList.begin();

    int sumNodes = 0;
    int sumBranches = 0;
    double sumDistances = 0;
    int sumTimes = 0;
    int sumSkelPoints = 0;

    if (nodeList.size() == branchList.size() && branchList.size() == distanceList.size() && distanceList.size() ==
                                                                                            timeList.size() &&
        timeList.size() == skeletonPointSingleCountList.size()) {
        for (; itNodes != nodeList.end() && itBranches != branchList.end() && itDistances != distanceList.end() && itTimes != timeList.end() &&
               itSkeletonPoints != skeletonPointSingleCountList.end(); itNodes++, itBranches++, itDistances++, itTimes++, itSkeletonPoints++) {
            sumNodes = sumNodes + *itNodes;
            sumBranches = sumBranches + *itBranches;
            sumDistances = sumDistances + *itDistances;
            sumTimes = sumTimes +  *itTimes;
            sumSkelPoints = sumSkelPoints + *itSkeletonPoints;
            //csvFile << *itNodes << "," << *itBranches << "," << *itDistances << "," << *itTimes << "," << *itSkeletonPoints << "\n";
        }
    }
    double avgDistances = sumDistances / distanceList.size();
    string inputFilename = imgfile.substr(14, (imgfile.length() - 18));

    //Write data in file
    ofstream csvFile(filenameSuffix, ios::app);
    csvFile << inputFilename << ";" << sumNodes << ";" << sumBranches << ";" << avgDistances << ";" << sumTimes << ";" << sumSkelPoints
    << ";" << skelPointsDistTrans << "\n";
    csvFile.close();
}
