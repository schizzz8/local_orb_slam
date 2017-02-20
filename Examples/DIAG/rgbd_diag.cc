
#include<iostream>
#include<algorithm>
#include<fstream>
#include <sstream>
#include<chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

#include <srrg_txt_io/message_reader.h>
#include <srrg_txt_io/message_writer.h>
#include <srrg_txt_io/pinhole_image_message.h>

#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace srrg_core;
typedef map<double,string> StampImageMap;
typedef map<double,Eigen::Isometry3f> StampIsometryMap;

void LoadImages(const string &input_filename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestampsRGB, vector<double> &vTimestampsDPT);

//void writeToFile(const string &input_filename, vector<double> &vTimestampsDPT, StampIsometryMap &camera_trajectory);
void writeToFile(const string &input_filename, vector<double> &vTimestampsDPT, ORB_SLAM2::System &slam);


int main(int argc, char **argv) {
    if(argc != 4){
        cerr << endl << "Usage: ./rgbd_diag path_to_vocabulary path_to_settings path_to_input_file" << endl;
        return 1;
    }

    cerr << "Vocabulary file path: " << argv[1] << endl;
    cerr << "Settings file path: " << argv[2] << endl;

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestampsRGB,vTimestampsDPT;
    string input_filename = string(argv[3]);
    LoadImages(input_filename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestampsRGB, vTimestampsDPT);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty()){
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()){
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],"DIAG.yaml",ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++){
        // Read image and depthmap from file
        imRGB = cv::imread("./"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread("./"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestampsDPT[ni];

        if(imRGB.empty()){
            cerr << endl << "Failed to load image at: "
                 << "./" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsRGB[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsRGB[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++){
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    //StampIsometryMap camera_trajectory;
    //SLAM.SaveKeyFrameTrajectoryDIAG(camera_trajectory);
    //cerr << camera_trajectory.size() << " camera poses\n";
    //cerr << endl;
    //writeToFile(input_filename,vTimestampsDPT,camera_trajectory);
    writeToFile(input_filename,vTimestampsDPT,SLAM);
    return 0;
}

void LoadImages(const string &input_filename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestampsRGB, vector<double> &vTimestampsDPT) {
    StampImageMap rgb_images;
    StampImageMap depth_images;
    MessageReader reader;
    reader.open(input_filename);
    double tolerance = 1.0/30.0;

    //Read the file and allocate map<timestamp,filename> for RGB and DEPTH images
    while (reader.good()) {
        BaseMessage* msg = reader.readMessage();
        if(!msg)
            continue;
        PinholeImageMessage* base_img=dynamic_cast<PinholeImageMessage*>(msg);
        if (! base_img)
            continue;
        if(strcmp(base_img->topic().c_str(),"/camera/rgb/image_raw") == 0)
            rgb_images.insert(std::pair<double,string> (base_img->timestamp(),base_img->binaryFullFilename()));
        if(strcmp(base_img->topic().c_str(),"/camera/depth/image_raw") == 0)
            depth_images.insert(std::pair<double,string> (base_img->timestamp(),base_img->binaryFullFilename()));
    }
    reader.close();

    //Find matching images
    StampImageMap::iterator it = rgb_images.begin();
    StampImageMap::iterator jt = depth_images.begin();
    while(it != rgb_images.end() && jt != depth_images.end()) {
        if(fabs(it->first - jt->first) < tolerance) {
            // Match found!
            vstrImageFilenamesRGB.push_back(it->second);
            vstrImageFilenamesD.push_back(jt->second);
            vTimestampsRGB.push_back(it->first);
            vTimestampsDPT.push_back(jt->first);
            //cerr << setprecision(16) << it->first << " " << it->second << " " << jt->first << " " << jt->second << endl;
            ++it;
            ++jt;
            continue;
        }
        if(it->first > jt->first + tolerance) {
            ++jt;
            continue;
        }
        ++it;
    }
}

//void writeToFile(const string &input_filename, vector<double> &vTimestampsDPT, StampIsometryMap &camera_trajectory) {
//    MessageReader reader;
//    reader.open(input_filename);
//    BaseMessage* msg = 0;
//    MessageWriter writer;
//    writer.open(input_filename.substr(0,input_filename.find("."))+"_with_trajectory.txt");

//    Eigen::Isometry3f global_transform = Eigen::Isometry3f::Identity();
//    Eigen::Isometry3f previous_transform;
//    bool first = true;
//    for(StampIsometryMap::iterator it = camera_trajectory.begin();it != camera_trajectory.end();it++){

//        double pose_stamp = it->first;
//        Eigen::Isometry3f current_transform = it->second;
////        cerr << pose_stamp << " " << current_transform.translation().transpose();
////        Eigen::Quaternionf rotation(current_transform.rotation());
////        cerr << " " << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << endl;

//        if(first){
//            previous_transform = current_transform;
//            first = false;
//            continue;
//        }

//        int idx=0;
//        double image_stamp;
//        bool found = false;
//        while (found == false) {
//            image_stamp = vTimestampsDPT[idx];
//            idx++;
//            if (pose_stamp - image_stamp == 0)
//                found = true;
//        }

//        found = false;
//        while(found == false) {
//            msg = reader.readMessage();
//            msg->untaint();
//            PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*> (msg);
//            if(img)
//                if((strcmp(img->topic().c_str(),"/camera/depth/image_raw") == 0) && (img->timestamp() == image_stamp)) {
//                    Eigen::Isometry3f delta_camera = previous_transform.inverse() * current_transform;
//                    Eigen::Isometry3f delta_robot = img->offset() * delta_camera * img->offset().inverse();
//                    global_transform = global_transform*delta_robot;
//                    img->setOdometry(global_transform);
//                    found = true;
//                    writer.writeMessage(*msg);
//                }
//        }
//        previous_transform = current_transform;
//    }
//}

void writeToFile(const string &input_filename, vector<double> &vTimestampsDPT, ORB_SLAM2::System &slam){
    MessageReader reader;
    reader.open(input_filename);
    BaseMessage* msg = 0;
    MessageWriter writer;
    writer.open(input_filename.substr(0,input_filename.find("."))+"_with_trajectory.txt");

    Eigen::Isometry3f global_transform = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f previous_transform;
    bool first = true;

    string trajectory_filename = "camera_trajectory.txt";
    slam.SaveKeyFrameTrajectoryTUM(trajectory_filename);

    ifstream trajectory_file;
    trajectory_file.open(trajectory_filename.c_str());
    string line;
    while(getline(trajectory_file,line)) {

        istringstream iss(line);
        double pose_stamp;
        float x,y,z,qx,qy,qz,qw;
        iss >> pose_stamp
                >> x >> y >> z
                >> qx >> qy >> qz >> qw;

        Eigen::Isometry3f current_transform = Eigen::Isometry3f::Identity();
        current_transform.translation() = Eigen::Vector3f (x,y,z);
        current_transform.rotate(Eigen::Quaternion<float> (qw,qx,qy,qz));

        if(first){
            previous_transform = current_transform;
            first = false;
            continue;
        }

        int idx=0;
        double image_stamp;
        bool found = false;
        while (found == false) {
            image_stamp = vTimestampsDPT[idx];
            idx++;
            if (pose_stamp - image_stamp == 0)
                found = true;
        }

        found = false;
        while(found == false) {
            msg = reader.readMessage();
            msg->untaint();
            PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*> (msg);
            if(img)
                if((strcmp(img->topic().c_str(),"/camera/depth/image_raw") == 0) && (img->timestamp() == image_stamp)) {
                    Eigen::Isometry3f delta_camera = previous_transform.inverse() * current_transform;
                    Eigen::Isometry3f delta_robot = img->offset() * delta_camera * img->offset().inverse();
                    global_transform = global_transform*delta_robot;
                    img->setOdometry(global_transform);
                    found = true;
                    writer.writeMessage(*msg);
                }
        }
        previous_transform = current_transform;
    }
}
