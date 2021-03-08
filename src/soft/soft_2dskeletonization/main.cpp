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
#include <fstream>
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
bool applyClosingToDapi = true;


//Declaration global values
std::string imgfile, skeletonImgName, prefix, resultFilename, toxin;
bool output = true;
double epsilon;
bool variableOutputNames;

/**
 *
 * @return a vector with all metadata lines
 */
vector <pair<string,string> > inputMetadata();

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
int inputFolderGrabbing(const char *directoryName, vector <pair<string,string> >  metadata);

/**
 *
 * @return
 */
Mat simpleRead(vector <pair<string,string> >  metadata);

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
void splitContours(Mat srcAlexa, Mat srcDAPI, vector <pair<string,string> >  metadata);

void writeCSVDataResult(list<int> nodeList, list<int> branchList, list<double> distanceList, list<int> timeList,
                        list<int> skeletonPointSingleCountList, int SkeletonPointsDist, int countNucleus,
                        string filenameSuffix);

Mat compareDistAndDapiFile(Mat dist, Mat dapi);

string changePointToComma(float number);

int countNucleus(Mat dapiInput);

Mat grayToBGR(Mat blue, Mat green, Mat red);

void generateCSVForIUF(string filename, double skeletonPoints, int nucleus, vector <pair<string,string> >, list<int> branchList, int nucleusArea, int maskedZytoplasmn);

vector<string> split(const string& str, const string& delim);


int main(int argc, char **argv) {
    //system("exec rm -r ../output/*");
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y/%H-%M");
    auto str = oss.str();
    prefix = str;
    system("../test.sh");
    system(" /opt/fiji/Fiji.app/ImageJ-linux64 -ij2 --headless --console -macro ../test3.ijm ../resources/");
    resultFilename = "../output/"+ prefix + "/resultData.csv";
    inputValuesRead(argc, argv);

    if (variableOutputNames) {
        cout << 224 <<endl;
        cout << filenameEnding <<endl;
        skeletonImgName = setVariableFilenames(filenameEnding, 0);
        cout << 226 <<endl;
    }
    cout << 229 <<endl;
    vector <pair<string,string> >  metadata = inputMetadata();
    cout << 228 <<endl;
    int result = inputFolderGrabbing("../resources", metadata);
    cout << "Programm fertig" <<endl;
    return result;
}

vector <pair<string,string> > inputMetadata(){
    //get number of lines
    pair <string, string> data;
    cout << 240 <<endl;
    vector <pair<string,string> > maskedUnmasked;
    cout << 242 <<endl;
    ifstream csvread;
    cout << 244 <<endl;

    csvread.open("../resources/metadata.csv", ios::in);
    cout << 247 <<endl;
    int i = 0;
    cout << 249 <<endl;
    if (csvread) {
        cout << 251 <<endl;
        //Read complete file and cut at ';'
        string s = "";
        cout << 254 <<endl;
        string masked = "";
        string unmasked = "";
        while (getline(csvread, s, '\n'))
        {
            cout << 259 <<endl;
            size_t index = s.find(";");
            cout << 261 <<endl;
            if (index != std::string::npos){
                cout << 263 <<endl;
                vector<string> v = split(s, ";");
                cout << 265 <<endl;
                cout << s <<endl;
                masked = v[0];
                unmasked = v[1];
                data = make_pair(masked,unmasked);
                maskedUnmasked.push_back(data);
            }
            else{
                std::string token = s.substr(0, s.length()-1);
                toxin = token;
            }
        }
        csvread.close();
    }
    else {
        cerr << "Fehler beim Lesen!" << endl;
    }
    return maskedUnmasked;
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

int inputFolderGrabbing(const char *directoryName, vector <pair<string,string> >  metadata){
    cout << 284 <<endl;
    DIR *dir;
    cout << 287<<endl;
    struct dirent *ent;
    cout << 289 <<endl;
    string dirName;
    cout << 291 <<endl;
    if ( openDirectory == true) {
        if ((dir = opendir(directoryName)) != NULL) {
            cout << 294 <<endl;
            while ((ent = readdir(dir))) {
                cout << 296 <<endl;
                dirName = ent->d_name;
                cout << 298 <<endl;
                if (dirName != "." && dirName != ".." && dirName != ".git") {
                    //picture data found
                  cout << 301 <<endl;  
                  if ((dirName.find(".png") != string::npos) && (dirName.find("Alexa488") != string::npos)){
                        imgfile = directoryName;
                        cout << imgfile << endl;
                        imgfile.append("/" + dirName);
                        cout << imgfile << endl;
                        Mat outClosing = simpleRead(metadata);
                    }
                    //directory found
                    else if (dirName.find(".") == string::npos)
                    {
                        string fullDirectoryName = directoryName;
                        fullDirectoryName.append("/" + dirName);
                        int n = fullDirectoryName.length();
                        char char_array[n+1];
                        strcpy (char_array, fullDirectoryName.c_str());
                        cout<<char_array<<endl;
                        const char *dirNeu = char_array;
                        inputFolderGrabbing(dirNeu, metadata);
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
        Mat outClosing = simpleRead(metadata);
    }
    //exit program
    return 0;
}

int inputValuesRead(int argc, char **argv) {
    boost::program_options::options_description desc("OPTIONS");
    desc.add_options()
            ("help", "Help message")
            ("imgfile",
             boost::program_options::value<std::string>(&imgfile)->default_value("../resources/" + inputImgDefault),
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
    cout << 375 <<endl;
    string first = imgfile.substr(13);
    cout<<imgfile<<endl;  

    int filenameLength = first.length();	
    string filename = first.substr(0, filenameLength - 4);	
    cout<<filename<<endl;

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
    cout<<generatedFilename<<endl;
    return generatedFilename;
}

cv::Mat simpleRead(vector <pair<string,string> >  metadata) {
    Mat matAlexaFile = imread(imgfile);
    if (matAlexaFile.empty()) {
        throw logic_error("Wrong input data Alexa file...");
    }
    string dapiFile = replaceSubstring(imgfile);
    Mat matDapiFile = imread(dapiFile);
    if (matDapiFile.empty()) {
        throw logic_error("Wrong input data DAPI file...");
    }
    //generateCSVForIUF( imgfile, 0, 0);
    splitContours(matAlexaFile, matDapiFile, metadata);
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
        cout << "Writing 'Complete' skeleton image to disc" << endl;
    }
    return image;
}

Mat generateSkeletonImage(Mat inputImage, shape::DiscreteShape<2>::Ptr dissh, skeleton::GraphSkel2d::Ptr grskelpropag,
                          int i) {
    displayopencv::DisplayGraphSkeleton(grskelpropag, inputImage, dissh->getFrame(),
                                        cv::Scalar(255, 255, 255));
    threshold(inputImage, inputImage, 0.4, 255, THRESH_BINARY);
    string filename = setVariableFilenames("-SkeletonImg.png", i);
    if (i == 0) {
        imwrite(filename, inputImage);
        cout << "Writing skeleton image to disc" << endl;
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
        cout << "Writing boundary image to disc" << endl;
    }
    return image;
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

void splitContours(Mat srcAlexa, Mat srcDAPI, vector <pair<string,string> >  metadata) {
    Mat kernel = (Mat_<float>(3, 3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);
    Mat imgLaplacian;
    filter2D(srcAlexa, imgLaplacian, CV_32F, kernel);
    Mat sharp;
    srcAlexa.convertTo(sharp, CV_32F);
    Mat imgResult = sharp - imgLaplacian;

    //convert 8B to greyscale
    imgResult.convertTo(imgResult, CV_8UC3);
    imgLaplacian.convertTo(imgLaplacian, CV_8UC3);

    //create binary image
    Mat bw;
    cvtColor(imgResult, bw, COLOR_BGR2GRAY);
    threshold(bw, bw, 40, 255, THRESH_BINARY | THRESH_OTSU);

    //imwrite("Binary_image.png", bw);
    resize(bw, bw, Size(bw.cols * 3, bw.rows * 3),0,0,INTER_NEAREST);
    Mat element = getStructuringElement(cv::MORPH_CROSS,Size(7,7),Point(-1,-1));
    morphologyEx(bw, bw, MORPH_CLOSE, element);
   
  
    //Morphological Closing for Dapi file 
	Mat DAPI_bw, bw_merged;
	if (applyClosingToDapi == true){
		srcDAPI.convertTo(DAPI_bw, CV_8UC3);
		cvtColor(DAPI_bw, DAPI_bw, COLOR_BGR2GRAY);
		threshold(DAPI_bw, DAPI_bw, 40, 255, THRESH_BINARY | THRESH_OTSU);
  
		resize(DAPI_bw, DAPI_bw, Size(DAPI_bw.cols * 3, DAPI_bw.rows * 3),0,0,INTER_NEAREST);
		Mat element_DAPI = getStructuringElement(cv::MORPH_CROSS,Size(15,15),Point(-1,-1));
		morphologyEx(DAPI_bw, DAPI_bw, MORPH_CLOSE, element_DAPI);

  
		//Merge and Save
		add(bw, DAPI_bw, bw_merged);
	}else{
		bw_merged = bw;
	}
  
    // Closing with inverted CROSS structuring element
	// See S-Modul Report from Frida for details and documentation on this
    Mat kernel_inverted_cross = (Mat_<uchar>(5,5) << 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, CV_8U);
    morphologyEx(bw_merged, bw_merged, MORPH_CLOSE, kernel_inverted_cross);
  
    imwrite("../output/Mopho_Output.png", bw_merged);

    // create CV_8U of distance image, needed for find conturs
    Mat dist_8u;
    bw_merged.convertTo(dist_8u, CV_8U);

    Mat dist = distanceTransformAlexa(bw_merged);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(dist_8u, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_L1);

    //Mat test = Mat::zeros(dist_8u.size(), CV_8UC3);
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
                if (area > 200) {
                    Mat singleContour = Mat::zeros(dist_8u.size(), CV_8UC3);
                    Scalar color(rand() & 255, rand() & 255, rand() & 255);
                    drawContours(singleContour, contours, (int) i, color, FILLED, 8, hierarchy);
                    drawContours(completeContour, contours, (int) i, color, FILLED, 8, hierarchy);
                    imwrite("../output/SingleContour.png", singleContour);
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
                    if (SkeletonPointsCounter != 0){
                        Mat imagepropag(dist_8u.size(), CV_8UC3, Scalar(255, 255, 255));
                        generateCompleteImage(completeIMG, dissh, disbnd, shppropag, grskelpropag, 0);
                        Mat boundImg = Mat::zeros(image.rows, image.cols, CV_8UC1);
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
            csvFile << "Dateiname;Anzahl Nodes;Anzahl Branches;Hausdorff Distance (px);Berechnungszeit (ms);Skelettpunkte Algorithmus;Skelettpunkte herunterskaliert;Skelettpunkte ohne DistanceTranform;Skelettpunkte ohne DistanceTranform herunterskaliert;Anzahl Zellkerne;Skelettpunkte (herunterskaliert) / Zellkerne;Skelettpunkte ohne Zytoplasma (herunterskaliert) / Zellkerne;\n";
            csvFile.close();
        }
		
        //
        SparseMat newMat(completeSkeleton);
        int SkeletonPointsCounterComplete = newMat.nzcount();
        consoleOutputCompleteData(SkeletonPointsCounterComplete);

        cvtColor(completeSkeleton, completeSkeleton, COLOR_BGR2GRAY);
        threshold(completeSkeleton, completeSkeleton, 1, 255, THRESH_BINARY | THRESH_OTSU);
        //only
        Mat compare = compareDistAndDapiFile(dist, srcDAPI);
        SparseMat maskedZytoplasmnBin(compare);
        int maskedZytoplasmn = maskedZytoplasmnBin.nzcount();


        Mat result = substractDistFromSkeletonfile(completeSkeleton, compare);
        SparseMat newMatResult(result);
        int skeletonPointsCounterCompleteWithoutDist = newMatResult.nzcount();
        int nucleusCounter = countNucleus(srcDAPI);
        writeCSVDataResult(nodeList, branchList, distanceList, timeList, skeletonPointSingleCountList,
                           skeletonPointsCounterCompleteWithoutDist, nucleusCounter, resultFilename);

        Mat completeWithoutDistanceTrans;
        cv::subtract(bw, result, completeWithoutDistanceTrans);
        string filename2 = setVariableFilenames("-CompleteWithoutDistanceTransform.png", 0);
        imwrite(filename2, completeWithoutDistanceTrans);

        Mat dist_8u_dapi;
        cvtColor(srcDAPI, dist_8u_dapi, COLOR_BGR2GRAY);
        resize(dist_8u_dapi, dist_8u_dapi, Size(dist_8u_dapi.cols * 3, dist_8u_dapi.rows * 3),0,0,INTER_NEAREST);
        Mat thres_dapi;
        threshold(dist_8u_dapi, thres_dapi, 200, 255, THRESH_BINARY);
        thres_dapi.convertTo(thres_dapi, CV_8UC1);
        SparseMat dapiBin(thres_dapi);
        int dapiArea = dapiBin.nzcount();

        generateCSVForIUF(imgfile, skeletonPointsCounterCompleteWithoutDist, nucleusCounter,metadata, branchList,
                dapiArea, maskedZytoplasmn);

        Mat multiChannel = grayToBGR(thres_dapi, bw_merged, result);

        Mat multiChannel2 = grayToBGR(result, bw_merged, thres_dapi);
        string filenameMultiChannelResult2 = setVariableFilenames("-ResultMultiChannel2.png", 0);
        imwrite(filenameMultiChannelResult2, multiChannel2);
        string filenameMultiChannelResult = setVariableFilenames("-ResultMultiChannel.png", 0);
        imwrite(filenameMultiChannelResult, multiChannel);

        Mat multiChannel3 = grayToBGR(bw_merged, thres_dapi, result);
        string filenameMultiChannelResult3 = setVariableFilenames("-ResultMultiChannel3.png", 0);
        imwrite(filenameMultiChannelResult3, multiChannel3);
    } else {
        throw logic_error("No contours found...");
    }
    //findContours(dist_8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
}

Mat compareDistAndDapiFile(Mat dist, Mat dapi){
    Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);
    imwrite("AlexaDist.png", dist);

    Mat dist_8u_dapi;
    Mat thres_dapi;

    dapi.convertTo(dist_8u_dapi, CV_8UC1);
    cvtColor(dist_8u_dapi, dist_8u_dapi, COLOR_BGR2GRAY);

    threshold(dist_8u_dapi, thres_dapi, 200, 255, THRESH_BINARY);
    imwrite("DapiThresNeu.png", thres_dapi);

    resize(thres_dapi, thres_dapi, Size(dist_8u_dapi.cols * 3, dist_8u_dapi.rows * 3),0,0,INTER_NEAREST);
    Mat resultArr = Mat::zeros(dist_8u.size(), CV_8UC1);

    if (dist_8u.rows == thres_dapi.rows && dist_8u.cols == thres_dapi.cols){
        vector<vector<Point> > contoursDist;
        vector<Vec4i> hierarchy;
        findContours(dist_8u, contoursDist, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_L1);

        if (!contoursDist.empty() && !hierarchy.empty()) {
            for (int i = 0; i < contoursDist.size(); i++) {
                vector<vector<Point> > contoursDapi;
                vector<Vec4i> hierarchy;
                findContours(thres_dapi, contoursDapi, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_L1);
                if (!contoursDapi.empty() && !hierarchy.empty()) {
                    for (int j = 0; j < contoursDapi.size(); j++) {
                        for (int k = 0; k < contoursDapi[j].size(); k++) {
                            Point_<int> p = contoursDapi[j][k];
                            double result = pointPolygonTest(contoursDist[i], p, true);
                            if(result >=0)
                            {
                                Scalar color(rand() & 255, rand() & 255, rand() & 255);
                                cv::drawContours(resultArr, contoursDist, (int) i, color, FILLED, 8);
                                break;
                            }
                        }
                    }
                }
            }
        }
        //imwrite("DistanceTransformResult.png", resultArr);
    }else{
        throw logic_error("Distance and Dapi file dont have the same size...");
    }
    Mat completeResult;
    cv::add(resultArr, thres_dapi, completeResult);
    return completeResult;
}

void writeCSVDataResult(list<int> nodeList, list<int> branchList, list<double> distanceList, list<int> timeList,
                        list<int> skeletonPointSingleCountList, int skelPointsDist, int countNucleus,  string filenameSuffix) {
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
        }
    }
    double avgDistances = sumDistances / distanceList.size();
    string inputFilename = imgfile.substr(14, (imgfile.length() - 18));
  
    float skelfaktor_wholeSkeleton = (sumSkelPoints / 4.4);
    string skelfaktor_wholeSkeletonStr = changePointToComma(skelfaktor_wholeSkeleton);
    float skelNucleus_wholeSkeleton = skelfaktor_wholeSkeleton / countNucleus;
    string skelNucleus_wholeSkeletonStr = changePointToComma(skelNucleus_wholeSkeleton);

    float skelfaktor = (skelPointsDist / 4.4);
    string skelfaktorStr = changePointToComma(skelfaktor);
    float skelNucleus = skelfaktor / countNucleus;
    string skelNucleusfaktorStr = changePointToComma(skelNucleus);

    //Write data in file
    ofstream csvFile(filenameSuffix, ios::app);
    csvFile << inputFilename << ";" << sumNodes << ";" << sumBranches << ";" << avgDistances << ";" << sumTimes << ";" << sumSkelPoints << ";" << skelfaktor_wholeSkeletonStr
    << ";" << skelPointsDist << ";" <<  skelfaktorStr << ";" << countNucleus << ";" << skelNucleus_wholeSkeletonStr << ";" << skelNucleusfaktorStr <<  "\n";
    csvFile.close();
}

string changePointToComma(float number){
    string str = to_string(number);
    size_t index = 0;
    index = str.find(".", index);
    if (index != std::string::npos )
    /* Make the replacement. */
        str.replace(index, 1, ",");
    return str;
}

int countNucleus(Mat dapiInput){
    Mat dist_8u_dapi;
    Mat thres_dapi;

    dapiInput.convertTo(dist_8u_dapi, CV_8UC1);
    cvtColor(dist_8u_dapi, dist_8u_dapi, COLOR_BGR2GRAY);

    threshold(dist_8u_dapi, thres_dapi, 200, 255, THRESH_BINARY);
    imwrite("DapiThres.png", thres_dapi);

    Mat dist;
    distanceTransform(thres_dapi, dist, DIST_L2, 3);
    normalize(dist, dist, 0, 1.0, NORM_MINMAX);
    imwrite("Distance Transform Image.png", dist);

    threshold(dist, dist, 0.4, 1.0, THRESH_BINARY);
    imwrite("Distance Transform Threshold Image.png", dist);

    // Dilate a bit the dist image
    Mat kernel1 = Mat::ones(3, 3, CV_8U);
    dilate(dist, dist, kernel1);
    imwrite("Peaks.png", dist);

    // Create the CV_8U version of the distance image
    // It is needed for findContours()
    Mat dist_8u2;
    dist.convertTo(dist_8u2, CV_8U);
    // Find total markers
    vector<vector<Point> > contours2;
    findContours(dist_8u2, contours2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int indx2 = contours2.size();

    // Create the marker image for the watershed algorithm
    Mat markers = Mat::zeros(dist.size(), CV_32S);
    // Draw the foreground markers
    for (size_t i = 0; i < contours2.size(); i++)
    {
        drawContours(markers, contours2, static_cast<int>(i), Scalar(static_cast<int>(i)+1), -1);
    }
    // Draw the background marker
    circle(markers, Point(5,5), 3, Scalar(255), -1);
    imwrite("Markers.png", markers*10000);
    return indx2;
}

Mat grayToBGR(Mat blue, Mat green, Mat red){
    Mat blueNew, greenNew, redNew;
    blueNew = blue *0.9;
    greenNew = green * 0.45;
    redNew = red;
    Mat out (blueNew.rows, blueNew.cols, CV_8UC3);
    Mat in[] = {blueNew, greenNew, redNew};
    int from_to[] = {0,0,1,1,2,2};
    mixChannels(&blueNew, 3, &out, 1, from_to, 3);
    return out;
}

 void generateCSVForIUF(string filename, double skeletonPoints, int nucleus, vector <pair<string,string> > metadata,
         list<int> branchList, int nucleusArea, int maskedZytoplasmn){

     vector<string> parthOfFile = split(filename, "/");
     int lenghtVector = parthOfFile.size();
     std::string path = parthOfFile[0] + "/output/" + prefix + "/";
     string resultFileIUF = path + "IUF.csv";
     ifstream file(resultFileIUF);
     //check if file not exists and creates one with headlines
     if(!file.good()){
         ofstream csvFile(resultFileIUF);
         csvFile << "Experiment ID ; " + toxin + " ; Well ; Sum neurite length ; Nucleus ; Relative neurite length ; Sum branches complete skelett ; Sum nucleus area ; Average nucleus area ; Masked zytoplasmn\n";
         csvFile.close();
     }

     list<int>::iterator itBranches = branchList.begin();
     int sumBranches = 0;
     for (; itBranches != branchList.end(); itBranches++) {
             sumBranches = sumBranches + *itBranches;
     }
     // create metadata

     std::string experiment = parthOfFile[lenghtVector-1].substr(0, parthOfFile[lenghtVector-1].find("_"));
     vector<string> fileNameParts = split(parthOfFile[lenghtVector-1], "_");
     string maskedConcentration = fileNameParts[3];
     string unmaskedConcentration;

     if(maskedConcentration.size() > 0){
         if(maskedConcentration != "R" && maskedConcentration != "ctrlctrl" && maskedConcentration != "ctrlDMSO"){
             for(int i = 0; i < metadata.size(); i++){
                 if(maskedConcentration == metadata[i].first)
                     if(metadata[i].second == "ctrlctrl"){
                         unmaskedConcentration = "Positive control (PC)";
                     }else if (metadata[i].second == "ctrlDMSO"){
                         unmaskedConcentration = "Solvent control (SC)";
                     } else{
                         unmaskedConcentration = metadata[i].second;
                     }
             }
         } else if (maskedConcentration == "R"){
             unmaskedConcentration = fileNameParts[4];
         } else if (maskedConcentration != "ctrlctrl"){
             unmaskedConcentration = "Positive control (PC)";
         }else{
             unmaskedConcentration = "Solvent control (SC)";
         }
     }
     double averageNeuriteLenght = 0;
     double averageNucleusArea = 0;
     string well = "C" + fileNameParts[5] + fileNameParts[7].substr(0, fileNameParts[7].find("."));
     if (nucleus != 0){
         averageNeuriteLenght = (skeletonPoints/4.4) / nucleus;
         averageNucleusArea = (nucleusArea / nucleus);
     }
     ofstream csvFile(resultFileIUF, ios::app);
     csvFile << experiment << ";" << unmaskedConcentration << ";" << well << ";" << (skeletonPoints/4.4) << ";" << nucleus << ";" << averageNeuriteLenght << ";" << sumBranches << ";" << nucleusArea << ";" << averageNucleusArea << ";" << maskedZytoplasmn <<"\n";
     csvFile.close();
}

vector<string> split(const string& str, const string& delim)
{
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}
