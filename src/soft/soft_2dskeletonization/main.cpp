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
#include <algorithm/skeletonization/BoundaryOperations.h>

using namespace std;
using namespace cv;
//TODO delete
//Declaration default values
string inputImgDefault = "test.png";
string skeletonImgNameDefault = "skeleton.png";
string filenameEnding = "-Epsilon1px-skeleton.png";
double epsilonValueDefault = 6.0;
bool outputDefault = true;
bool variableOutputNamesDefault = true;


//Declaration globale values
std::string imgfile, skeletonImgName, prefix;
bool output = false;
double epsilon;
bool variableOutputNames;
bool openDirectory = true;


Mat simpleReadAndConvertBW();

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

tuple<double, double, int, int> EvalSkel(const shape::DiscreteShape<2>::Ptr dissh,
                                         const boundary::DiscreteBoundary<2>::Ptr disbnd,
                                         const skeleton::GraphSkel2d::Ptr skel);

int inputFolderGrabbing(const char *directoryName);

cv::Mat simpleRead();

/**
 * Print the Skeleton Counter for the hole Image
 * @param skeletonPointsCounter
 */
void consoleOutputCompleteData(int skeletonPointsCounter);

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
generateBoundaryImage(Mat image, shape::DiscreteShape<2>::Ptr dissh, boundary::DiscreteBoundary<2>::Ptr disbnd, int i);

/**
 * Generates a list with a pair of x and y coordinates of all skeleton point
 * @param img image with only the skeleton points
 * @return List with x and y coordinate pairs
 */
vector<pair<int, int>> getAllImageCoordinates(Mat img);

/**
 * Writes the data from the given list into csv data
 * @param skeletonPoints list of x and y coordinates for the skeleton data
 * @param filenameSuffix String for the filename suffix
 */
void writeCSVData(vector<pair<int, int>> skeletonPoints, string filenameSuffix, int i);

void splitContours(Mat src);

vector<pair<int, int>> getListFromPicture(Mat pic);

void writeCSVDataResult(list<int> nodeList, list<int> branchList, list<double> distanceList, list<int> timeList,
                        list<int> skeletonPointSingleCountList, string filenameSuffix);

void getClosestInd(skeleton::GraphSkel2d::Ptr grskelpropag, const boundary::DiscreteBoundary<2>::Ptr disbnd);

int main(int argc, char **argv) {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y/%H:%M");
    auto str = oss.str();
    prefix = str;
    system("../test.sh");

    inputValuesRead(argc, argv);

    if (variableOutputNames) {
        skeletonImgName = setVariableFilenames(filenameEnding, 0);
    }

    int result = inputFolderGrabbing("../ressources");
  
    system("chmod -R 777 ../output");
  
    cout << "fertig" <<endl;
    return result;
}

vector<pair<int, int>> getListFromPicture(Mat pic, int i, int j, vector<pair<int, int>> &list) {
    cv::Size s = pic.size();

    if (pic.at<uchar>(i + 1, j) != 0) {
        pair<int, int> pair = make_pair(i + 1, j);
        if ((std::find_if(list.begin(), list.end(), [&pair](const std::pair<int, int> &el) {
            return el.first == pair.first && el.second == pair.second;
        }) == list.end())) {
            list.push_back(pair);
            return getListFromPicture(pic, i + 1, j, list);
        }
    }
    if (pic.at<uchar>(i, j + 1) != 0) {
        pair<int, int> pair = make_pair(i, j + 1);
        if ((std::find_if(list.begin(), list.end(), [&pair](const std::pair<int, int> &el) {
            return el.first == pair.first && el.second == pair.second;
        }) == list.end())) {
            list.push_back(pair);
            return getListFromPicture(pic, i, j + 1, list);
        }
    }
    if (pic.at<uchar>(i - 1, j) != 0) {
        pair<int, int> pair = make_pair(i - 1, j);
        if ((std::find_if(list.begin(), list.end(), [&pair](const std::pair<int, int> &el) {
            return el.first == pair.first && el.second == pair.second;
        }) == list.end())) {
            list.push_back(pair);
            return getListFromPicture(pic, i - 1, j, list);
        }
    }
    if (pic.at<uchar>(i, j - 1) != 0) {
        pair<int, int> pair = make_pair(i, j - 1);
        if ((std::find_if(list.begin(), list.end(), [&pair](const std::pair<int, int> &el) {
            return el.first == pair.first && el.second == pair.second;
        }) == list.end())) {
            list.push_back(pair);
            return getListFromPicture(pic, i, j - 1, list);
        }
    }
    return list;
}

vector<pair<int, int>> getListFromPicture(Mat pic) {

    int rows = pic.rows;
    int cols = pic.cols;

    cv::Size s = pic.size();
    rows = s.height;
    cols = s.width;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            int x = pic.at<uchar>(i, j);
            if (x != 0) {
                vector<pair<int, int>> liste = vector<pair<int, int>>();
                liste.emplace_back(i, j);
                return getListFromPicture(pic, i, j, liste);
            }
        }
    }
    return vector<pair<int, int>>();
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

cv::Mat simpleReadAndConvertBW() {
    Mat shpimggray = imread(imgfile);
    //cout << shpimggray << endl;
    if (shpimggray.empty()) {
        throw logic_error("Wrong input data...");
    }
    splitContours(shpimggray);
    /*Mat shpimg;
    threshold(shpimggray,shpimg,1,255,THRESH_BINARY);

    string filename = setVariableFilenames("-ThresholdOutput.png");
    imwrite(filename, shpimg);

//     topological closure of the binary shape
    Mat outClosing;
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3),Point(1,1));
    morphologyEx(shpimg, outClosing, MORPH_CLOSE, element);
    filename = setVariableFilenames("-ClosingOutput.png");
    imwrite(filename, outClosing);*/

    return shpimggray;
}

tuple<double, double, int, int> EvalSkel(const shape::DiscreteShape<2>::Ptr dissh,
                                         const boundary::DiscreteBoundary<2>::Ptr disbnd,
                                         const skeleton::GraphSkel2d::Ptr skel) {
    shape::DiscreteShape<2>::Ptr shp(new shape::DiscreteShape<2>(dissh->getWidth(), dissh->getHeight()));
    algorithm::skinning::Filling(shp, skel);

    double res = algorithm::evaluation::SymDiffArea(dissh, shp);
    double res2 = algorithm::evaluation::HausDist(skel, disbnd, dissh->getFrame());

    //ofstream exp;
    //string fileNameDegreeOut = "../output/"+prefix+"nodeDegrees.csv";
    //exp.open(fileNameDegreeOut);

    list<unsigned int> lnod;
    skel->getAllNodes(lnod);
    unsigned int nbbr = 0;
    unsigned int currentNodeID = 0;
    for (std::list<unsigned int>::iterator it = lnod.begin(); it != lnod.end(); it++) {
        unsigned int deg = skel->getNodeDegree(*it);
        if (deg != 2)
            nbbr += deg;

        //exp << currentNodeID << ";" << deg << ";" << "\n";
        //currentNodeID = currentNodeID+1;
    }
    nbbr /= 2;
    tuple<double, double, int, int> result = std::make_tuple(res * 100.0, res2, skel->getNbNodes(), nbbr);

    //exp.flush();
    //exp.close();

    return result;
}

int inputFolderGrabbing(const char *directoryName){
    DIR *dir;
    struct dirent *ent;
    string dirName;
    if ( openDirectory == true) {
        if ((dir = opendir(directoryName)) != NULL) {
            while ((ent = readdir(dir))) {
                dirName = ent->d_name;
                if (dirName != "." && dirName != ".." && dirName != ".git") {
                    //picture data found
                    if (dirName.find(".png") != string::npos){
                        imgfile = directoryName;
                        imgfile.append("/" + dirName);
                        Mat outClosing = simpleRead();
                    }
                        //directory found
                    else if (dirName.find(".") == string::npos)
                    {
                        string fullDirectoryName = directoryName;
                        fullDirectoryName.append("/" + dirName);
                        int n = fullDirectoryName.length();
                        char char_array[n+1];
                        strcpy (char_array, fullDirectoryName.c_str());
                        const char *dirNeu = char_array;
                        inputFolderGrabbing(dirNeu);
                    }
                    else {
                        //nothing
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

cv::Mat simpleRead() {
    Mat matAlexaFile = imread(imgfile);
    if (matAlexaFile.empty()) {
        throw logic_error("Wrong input data...");
    }
    try
    {
    splitContours(matAlexaFile);
    }
    catch(...)
    {
      //no nothing
    }
    return matAlexaFile;
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

    if (output && i == 0 || i != 859) {
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
    if (i == 0 || i != 859) {
        imwrite(filename, inputImage);
    }
    return inputImage;
}

Mat
generateBoundaryImage(Mat image, shape::DiscreteShape<2>::Ptr dissh, boundary::DiscreteBoundary<2>::Ptr disbnd, int i) {
    displayopencv::DisplayDiscreteBoundary(disbnd, image, dissh->getFrame(),
                                           cv::Scalar(255, 255, 255));
    string filename = setVariableFilenames("-BoundaryImg.png", i);
    if (i == 0 || i != 859) {
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

void writeCSVData(vector<pair<int, int>> skeletonPoints, string filenameSuffix, int i) {
    string csvFilename = setVariableFilenames(filenameSuffix, i);
    ofstream csvFile(csvFilename);
    for (pair<int, int> p : skeletonPoints) {
        csvFile << p.first << ", " << p.second << "\n";
    }
    csvFile.close();
}



void splitContours(Mat src) {
    Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
    Mat imgLaplacian;
    filter2D(src, imgLaplacian, CV_32F, kernel);
    Mat sharp;
    src.convertTo(sharp, CV_32F);
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
    //resize(bw, bw, Size(bw.cols * 3, bw.rows * 3));
//    Mat element = getStructuringElement(cv::MORPH_RECT,Size(3,3),Point(1,1));
//    morphologyEx(bw, bw, MORPH_DILATE, element);
//    morphologyEx(bw, bw, MORPH_CLOSE, element);
//    morphologyEx(bw, bw, MORPH_DILATE, element);
    imwrite("../output/Mopho_Output.png", bw);

//    Mat dist;
//    distanceTransform(bw, dist, DIST_L2, 3);
//    imwrite("../output/Distance.png", dist);


    // create CV_8U of distance image, needed for find conturs
    Mat dist_8u;
    bw.convertTo(dist_8u, CV_8U);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(dist_8u, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_L1);

    Mat test = Mat::zeros(dist_8u.size(), CV_8UC3);
    std::string str;

//    for(size_t i = 0; i< contours.size(); i++){
//        Scalar color (rand()&255, rand()&255, rand()&255);
//        drawContours(test, contours, (int) i, color, LINE_4, 8, hierarchy);
//        std::string str = std::to_string(i);
//        string filenameTest = "test";
//        filenameTest.append(str);
//        filenameTest.append(".png");
//        imwrite(filenameTest , test );
//    }
//    imwrite("complete.png", test);

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
                if (indx != 100000 && !(area <=1)) {
                    Mat singleContour = Mat::zeros(dist_8u.size(), CV_8UC3);
                    Scalar color(rand() & 255, rand() & 255, rand() & 255);
                    drawContours(singleContour, contours, (int) i, color, FILLED, 8, hierarchy);
                    drawContours(completeContour, contours, (int) i, color, FILLED, 8, hierarchy);
                    imwrite("../output/SingleContour.png", singleContour);

                    threshold(singleContour, singleContour, 1, 255, THRESH_BINARY);
                    cvtColor(singleContour, singleContour, COLOR_BGR2GRAY);
//                    imwrite("../output/AfterThreshold.png", singleContour);
//
//                    morphologyEx(singleContour, singleContour, MORPH_CLOSE, element);
//                    morphologyEx(singleContour, singleContour, MORPH_DILATE, element);
//                    morphologyEx(singleContour, singleContour, MORPH_DILATE, element);
//                    morphologyEx(singleContour, singleContour, MORPH_CLOSE, element);
                    //morphologyEx(singleContour, singleContour, MORPH_DILATE, element);
//                    imwrite("../output/Mopho_Output.png", singleContour);

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


                    Mat imagepropag(dist_8u.size(), CV_8UC3, Scalar(255, 255, 255));
                    generateCompleteImage(imagepropag, dissh, disbnd, shppropag, grskelpropag, indx);
                    generateCompleteImage(completeIMG, dissh, disbnd, shppropag, grskelpropag, 0);

                    Mat skelImg = Mat::zeros(image.rows, image.cols, CV_8UC1);
                    Mat skeletonImg = generateSkeletonImage(skelImg, dissh, grskelpropag, indx);
                    generateSkeletonImage(completeSkeleton, dissh, grskelpropag, 0);

                    Mat boundImg = Mat::zeros(image.rows, image.cols, CV_8UC1);
                    Mat boundaryImg = generateBoundaryImage(boundImg, dissh, disbnd, indx);

                    generateBoundaryImage(completeBoundary, dissh, disbnd, 0);

                    auto boundaryPointList = getListFromPicture(boundaryImg);
                    writeCSVData(boundaryPointList, "-boundarypoints.csv", indx);


                    vector<pair<int, int>> skelPointsList = getAllImageCoordinates(skeletonImg);
                    map<pair<int, int>, vector<vector<pair<int, int>>>> contractlist2;
                    for (auto& t : contractlist){
                        auto f = t.first;
                        if((std::find(skelPointsList.begin(), skelPointsList.end(), f) != skelPointsList.end())){
                            contractlist2.insert({f, t.second});
                        }else{
                           // std::cout << "found: " << f.first << "|" << f.second << std::endl;
                        }
                    }
                    
                    ofstream exp;
                    string filenameContractSet = "../output/" + prefix + "/contractSet.csv";
                    exp.open(filenameContractSet);

                    for(auto& t : contractlist2){
                        auto first = t.first;
                        auto list = t.second;
                        bool first1 = true;
                        exp << "(" << first.first << ", " << first.second << ");";
                        for(auto& list2 : list) {
                            for(auto& e : list2) {
                                exp << "(" << e.first << ", " << e.second << ")";
                            }
                            exp << ";";
                        }
                        exp << std::endl;

                    }
                    exp.flush();
                    exp.close();
                    vector<pair<int, int>> boundaryPointsList = getAllImageCoordinates(boundaryImg);
                    SparseMat newMat(skeletonImg);
                    int SkeletonPointsCounter = newMat.nzcount();
                    writeCSVData(skelPointsList, "-skeletonpoints.csv", indx);
                    //consoleOutputSingleData(respropag, t0, SkeletonPointsCounter);

                    nodeList.push_back(get<2>(respropag));
                    branchList.push_back(get<3>(respropag));
                    distanceList.push_back(get<1>(respropag));
                    timeList.push_back(t0);
                    skeletonPointSingleCountList.push_back(SkeletonPointsCounter);
                    writeCSVDataResult(nodeList, branchList, distanceList, timeList, skeletonPointSingleCountList,
                                       "_generalData.csv");
                    getClosestInd(grskelpropag, disbnd);
                    indx++;
                }
            }
        }
        SparseMat newMat(completeSkeleton);
        int SkeletonPointsCounterComplete = newMat.nzcount();
        consoleOutputCompleteData(SkeletonPointsCounterComplete);
        imwrite("completeContour.png", completeContour);
    } else {
        throw logic_error("No contours found...");
    }

    findContours(dist_8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}

void writeCSVDataResult(list<int> nodeList, list<int> branchList, list<double> distanceList, list<int> timeList,
                        list<int> skeletonPointSingleCountList, string filenameSuffix) {
    string csvFilename = setVariableFilenames(filenameSuffix, 0);
    ofstream csvFile(csvFilename);
    csvFile << "Anzahl Nodes , Anzahl Branches , Hausdorff Distance (px), Berechnungszeit (ms), Skelettpunkte \n";
    list<int>::iterator it1 = nodeList.begin();
    list<int>::iterator it2 = branchList.begin();
    list<double>::iterator it3 = distanceList.begin();
    list<int>::iterator it4 = timeList.begin();
    list<int>::iterator it5 = skeletonPointSingleCountList.begin();
    if (nodeList.size() == branchList.size() && branchList.size() == distanceList.size() && distanceList.size() ==
                                                                                            timeList.size() &&
        timeList.size() == skeletonPointSingleCountList.size()) {
        for (; it1 != nodeList.end() && it2 != branchList.end() && it3 != distanceList.end() && it4 != timeList.end() &&
               it5 != skeletonPointSingleCountList.end(); it1++, it2++, it3++, it4++, it5++) {
            csvFile << *it1 << "," << *it2 << "," << *it3 << "," << *it4 << "," << *it5 << "\n";
        }
    }
    csvFile.close();
}


void getClosestInd(skeleton::GraphSkel2d::Ptr grskelpropag, const boundary::DiscreteBoundary<2>::Ptr disbnd) {

// creation of the optimized boundary structure
    algorithm::skeletonization::propagation::OptiBnd optiBnd;
    algorithm::skeletonization::propagation::OptiUsedBnd optiUsedBnd;
    createOptiBnd(disbnd, optiBnd, optiUsedBnd);
    std::list<unsigned int> lind;
    grskelpropag->getAllNodes(lind);
  
    writeCSVData(optiBnd, "_optiboundary.csv", 0);
    
    string csvFilename = setVariableFilenames("_skeletondata.csv", 0);
    ofstream csvFile(csvFilename);
    for (auto x : lind) {
        std::list<unsigned int> closest;
        auto y = grskelpropag->getNode<mathtools::geometry::euclidian::HyperSphere<2>>(x);
        auto z = y.getCenter().getCoords();
        auto r = y.getRadius();
//        std::cout << z[0] << "/" << z[1] << ": ";
        //csvFile << x << ";";
        csvFile <<  z[0] << "," << z[1] << ";";
        csvFile << r << ";";
        
        unsigned int deg = grskelpropag->getNodeDegree(x);
        csvFile << deg << ";";
      
        list<unsigned int> nbr;
        grskelpropag->getNeighbors(x,nbr);
      
        for (auto ind : nbr){
              csvFile << ind << ",";
        }
        csvFile << ";";

                  algorithm::skeletonization::propagation::closestInd(optiBnd, z, closest);
        for (auto w : closest){
            auto g = optiBnd[w];
//            std::cout << g.coords[0] << "/" << g.coords[1] << "; ";
            csvFile <<  g.coords[0] << "," << g.coords[1] << "|";
        }
        csvFile << "; \n";
//        std::cout << std::endl;
    }
    csvFile.close();
}
