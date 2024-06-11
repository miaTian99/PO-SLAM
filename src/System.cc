/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include "Object.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

// #include "include/Semantic.h"       // [EAO] online yolo.

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

using namespace Eigen;

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, 
               const string &flag, const eSensor sensor, const string &seqName,
               const bool bUseViewer):  mSensor(sensor),
                                        mbReset(false),
                                        mbActivateLocalizationMode(false),
                                        mbDeactivateLocalizationMode(false),
                                        bSemanticOnline(false)  // [EAO] online/offline yolo detection.
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    //bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // [EAO] online / offline yolo detection.
    if(bSemanticOnline)
    {
        std::cout << "Online Semantic mode" << std::endl;
        // TODO Load network
    }
    else
    {
        std::cout << "Offline Semantic mode" << std::endl;
    }

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, flag, seqName);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR, flag); // EAO->flag
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    // mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, flag); 
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize
    mpSemiDenseMapping = new ProbabilityMapping(mpMap); 
    // 半稠密建图线程
    mptSemiDense = new thread(&ProbabilityMapping::Run, mpSemiDenseMapping);

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile, flag);
    if(bUseViewer){
        mptViewer = new thread(&Viewer::Run, mpViewer);  
    }
    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);
	// give tracker semi dense pointer, for resetting
	mpTracker->SetSemiDenseMapping(mpSemiDenseMapping);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // // [EAO] Set pointers between threads.
    // if(mbSemanticOnline)
    // {
    //     mpTracker->SetSemanticer(mpSemanticer);
    // }
    // if(mbSemanticOnline)
    // {
    //     mpSemanticer->SetTracker(mpTracker);
    // }
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   
    
    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }
    
    // wait for semi dense thread
    {
        unique_lock<mutex> lock(mpSemiDenseMapping->mMutexSemiDense);
    } 
    // 返回的当前关键帧的位姿
    // TODO：（objPose不变的可能的解决方案之一）从这里获取mpMap->GetPose的结果再返回，相当于执行结束GrabImageStereo再执行位姿获取，然后再返回
    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp,bSemanticOnline);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    // wait for semi dense thread
    {
        unique_lock<mutex> lock(mpSemiDenseMapping->mMutexSemiDense);
    }

    return mpTracker->GrabImageMonocular(im, timestamp, bSemanticOnline);
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

// 默认：启用三维重建，保存三维重建和相对导航结果。如果不存储三维重建和轨迹结果，则注释以下代码
void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpSemiDenseMapping->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA() || !mpSemiDenseMapping->isFinished())
    // while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {   
        // cout << "check hung up thread:" << endl;
        // cout << "mpLocalMapper: " << mpLocalMapper->isFinished() << endl;
        // cout << "mpLoopCloser Finish: " << mpLoopCloser->isFinished() << endl;
        // cout << "mpLoopCloser GBA: " << mpLoopCloser->isRunningGBA() << endl;
        // cout << "mpSemiDenseMapping: " << mpSemiDenseMapping->isFinished() << endl;
        usleep(5000);
    }
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

// 如果不存储三维重建和轨迹结果，则启用以下注释代码
// void System::Shutdown()
// {
//     mpLocalMapper->RequestFinish();
//     mpLoopCloser->RequestFinish();
//     //mpViewer->RequestFinish();
//     mpSemiDenseMapping->RequestFinish();

//     // Wait until all thread have effectively stopped
//     while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
//           !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA() || !mpSemiDenseMapping->isFinished())
//     {
//         usleep(5000);
//     }

//     pangolin::BindToContext("ORB-SLAM2: Map Viewer");
// }



void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        // f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        // for evo_eval 
        f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

//******start added from SLAM3 - revised according to SLAM2******//
void System::SaveTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving trajectory of map to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    int numMaxKFs = 0;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;


        ORB_SLAM2::KeyFrame* pKF = *lRit;
        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
        //cout << "KF: " << pKF->mnId << endl;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        Trw = Trw * pKF->GetPose()*Two; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

// 自定义：物体相对位姿估计轨迹存储
void System::SaveObjPose(const string &filename)
{
    cout << endl << "Saving estimated global pose of object to " << filename << " ..." << endl;

    int numMaxKFs = 0;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    // TODO: 搞清楚这里的地图维护问题
    vector<Object_Map*> vpMPs = mpMap->GetObjects();  //tyl 这里存储的mpMap是全局还是最终的地图？？？
    
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    int obj_index = 0;
    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;


        ORB_SLAM2::KeyFrame* pKF = *lRit;
        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
        //cout << "KF: " << pKF->mnId << endl;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        if(vpMPs.size()==0){
            cout << "vpMPs are empty!! " << endl; //debug
            continue;
        }

        // TODO：存在一定的逻辑问题，可能是导致objMap维护的值
        Object_Map* objMap = vpMPs[0];
        if(vpMPs.size()>0){  
            int bestCon = vpMPs[0]->mnConfidence;
            for (int i = 0; i < vpMPs.size(); i++){
                if (objMap->bBadErase)
                    continue;
                // map object.
                if(vpMPs[i]->mnConfidence > bestCon)
                    objMap = vpMPs[i];
            }
        }
        // world frame to get object pose
        // cv::Mat Twq = Converter::toCvMat(objMap->mCuboid3D.pose);
        // cv::Mat Rwc = Twq.rowRange(0,3).colRange(0,3).t();
        // cv::Mat twc = -Rwc*Twq.rowRange(0,3).col(3);

        // change the relative relationship: first frame pose -> object pose

        // cv::Mat Twq = Converter::toCvMat(objMap->mCuboid3D.pose);
        // float c_x = objMap->mCuboid3D.cuboidCenter(0);
        // float c_y = objMap->mCuboid3D.cuboidCenter(1);
        // float c_z = objMap->mCuboid3D.cuboidCenter(2);

        // float s_l_half = objMap->mCuboid3D.lenth/2;
        // float s_w_half = objMap->mCuboid3D.width/2;
        // float s_h_half = objMap->mCuboid3D.height/2;
        // // 起始点平移到物体所在位置
        // cv::Mat Two_ = Two.clone();
        // Two_.at<float>(0,3) = Twq.at<float>(0,3);
        // Two_.at<float>(1,3) = Twq.at<float>(1,3);
        // Two_.at<float>(2,3) = Twq.at<float>(2,3);
        // cout << "position of Two: " << Two.at<float>(0,3) << " " << Two.at<float>(1,3) << " " << Two.at<float>(2,3) << endl;
        // cout << "position of Twq: " << Twq.at<float>(0,3) << " " << Twq.at<float>(1,3) << " " << Twq.at<float>(2,3) << endl;
        // cout << "centoid_x: " << c_x << " centoid_y: " << c_y << " centoid_z: "<< c_z << endl;
        // cout << "s_w_half: " << s_w_half << " s_l_half: " << s_l_half << " s_h_half: "<< s_h_half << endl;
        // Trw = Trw * pKF->GetPose()*Two_; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        Trw = Trw * pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw; 
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();  // 当前相机相对于第一帧的旋转矩阵(取Twc[R]=Twc[R].T)
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);        // 当前相机相对于第一帧的平移向量(取Twc[t]=Twc[R]*Tcw[-t])
        cout << "当前相机相对于第一帧的位姿Twc: " << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << endl;

        cv::Mat Twq = Converter::toCvMat(objMap->mCuboid3D.pose);
        float c_x = objMap->mCuboid3D.cuboidCenter(0);
        float c_y = objMap->mCuboid3D.cuboidCenter(1);
        float c_z = objMap->mCuboid3D.cuboidCenter(2);

        float s_l_half = objMap->mCuboid3D.lenth/2;
        float s_w_half = objMap->mCuboid3D.width/2;
        float s_h_half = objMap->mCuboid3D.height/2;
        // 物体位在当前帧相机下的位姿  =  第一帧在当前相机下的位姿Tcw * 物体在第一帧的下位姿Twq
        cv::Mat Tcq = Tcw*Twq;
        cout << "物体在第一帧的下位姿Twq: " << Twq.at<float>(0,3) << " " << Twq.at<float>(1,3) << " " << Twq.at<float>(2,3) << endl;
        cout << "物体在当前帧相机下的位姿Tcq: " << Tcq.at<float>(0,3) << " " << Tcq.at<float>(1,3) << " " << Tcq.at<float>(2,3) << endl;
        cv::Mat Rcq = Tcq.rowRange(0,3).colRange(0,3);  // Tcq旋转矩阵
        cv::Mat tcq = Tcq.rowRange(0,3).col(3);         // Tcq平移向量
        cout << "centoid_x: " << c_x << " centoid_y: " << c_y << " centoid_z: "<< c_z << endl;
        cout << "s_w_half: " << s_w_half << " s_l_half: " << s_l_half << " s_h_half: "<< s_h_half << endl;

       f << setprecision(9) << Rcq.at<float>(0,0) << " " << Rcq.at<float>(0,1)  << " " << Rcq.at<float>(0,2) << " "  << tcq.at<float>(0) << " " <<
             Rcq.at<float>(1,0) << " " << Rcq.at<float>(1,1)  << " " << Rcq.at<float>(1,2) << " "  << tcq.at<float>(1) << " " <<
             Rcq.at<float>(2,0) << " " << Rcq.at<float>(2,1)  << " " << Rcq.at<float>(2,2) << " "  << tcq.at<float>(2) << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

//******end added from SLAM3 - revised according to SLAM2******//

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

} //namespace ORB_SLAM
