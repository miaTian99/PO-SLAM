/**
 * Description : Redis + orbslam2_双目执行程序脚本
 * Date :  26/06/2023
 * Author : Mia Tien
 * Modified : 29/08/2023
**/

#include<sw/redis++/redis++.h>
#include<iostream>
#include<fstream>
#include<unistd.h>
#include<sys/stat.h>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include"../build/offline_bbox.pb.h"
using namespace std;
using namespace sw::redis;
#define COMPILEDWITHC11

//接口回调函数
void call_back(std::string channel, std::string msg)
{
    if (channel=="offline_bbox")
    {
        cout << "channel:" << channel << endl;
        offline_bbox::offline_bboxes bboxes;
        bboxes.ParseFromString(msg);
        string timestamp = bboxes.time_sec();
        string seq_name = bboxes.seq_name(); 
        string update_freq = bboxes.update_freq();
        // 解析RGB图像
        vector<uchar> image_data(bboxes.image_data().begin(),bboxes.image_data().end());
        // 边界检查
        if (image_data.size()==0)  
        {
            cout << "No SAM Mask is transmited !"<< endl;
            return ;
        }else{
            cout << image_data.size() << endl;
        } 
        cv::Mat sam_mask = cv::Mat(image_data);
        //sam_mask = sam_mask.reshape(3,480); 
        sam_mask = sam_mask.reshape(3,1024);                  // 第一个参数：3通道，第二维：行数1024（列数自动确定）
        cv::cvtColor(sam_mask, sam_mask, cv::COLOR_RGB2BGR);  // matplotlib对应的RGB转opencv对应的BGR
        //cv::imshow("sam_mask", sam_mask);
        //cv::waitKey(1);
        // 存储收到的数据，若文件不存在则创建文件
        string rootdir = "/home/ai-i-tianyaolin/workspace/EAO-SLAM-v1/dataset/SePT/" + seq_name + "/mav0/";
        string mask_save_path = rootdir + "/sam_mask_uf" + update_freq;
        string bbox_save_path = rootdir + "/semantic_bboxes_uf" + update_freq;
        
        if(access(rootdir.c_str(), 0)==-1){
            mkdir(rootdir.c_str(), 0777); 
        }
        if(access(mask_save_path.c_str(), 0)==-1){
            mkdir(mask_save_path.c_str(), 0777); 
        }
        if(access(bbox_save_path.c_str(), 0)==-1){
            mkdir(bbox_save_path.c_str(), 0777); 
        }

        cv::imwrite(mask_save_path + "/"+ timestamp + ".png", sam_mask);
        string file = bbox_save_path + "/" + timestamp + ".txt";
        //fstream outfile(file, ios::out | ios::app); //ios::app追加新记录,“a”(离线版需要)
        fstream outfile(file, ios::out);
        if (!outfile.is_open())
        {   
            //文件不存在
            cout << "image_matching file open fail" << endl;
            //创建文件
            //ofstream outfile(file, ios::out | ios::app); //ios::app追加新记录,“a”(离线版需要)
            ofstream outfile(file, ios::out);
            if (outfile) cout << "new file created" << endl;
            exit(0);
        }
        for (int i = 0; i < bboxes.bbox_size(); i++){
            const offline_bbox::Bbox obj_bbox = bboxes.bbox(i);
            //接收数据流
            int obj_id = obj_bbox.obj_id(), bbox_x = obj_bbox.bbox_x(), bbox_y = obj_bbox.bbox_y(), 
                bbox_w = obj_bbox.bbox_w(), bbox_h = obj_bbox.bbox_h();
            float bbox_p = obj_bbox.bbox_p(); 
            //转为字符串
            string line;
            line = to_string(obj_id) + ' ' + to_string(bbox_x) + ' ' 
                + to_string(bbox_y) + ' ' + to_string(bbox_w) + ' '
                 + to_string(bbox_h) + ' ' + to_string(bbox_p);
            //将当前单条检测框信息写入文件末尾 
            outfile << line << endl;
            cout << line << endl;
            line.clear();
        }
        outfile.close();
    }
}

        

int main()
{
    Redis redis("tcp://127.0.0.1:6379");
    auto sub1=redis.subscriber();
    string channel="offline_bbox";
    sub1.subscribe(channel);
    sub1.on_message(call_back);
    //cout<<"sub1 subscribe channel:"<<channel<<endl;
    cout<<"Begin to listen..."<<endl;

    while (true) 
    {
        try 
        {
            sub1.consume();
        } 
        catch (const Error &err) 
        {
            // Handle exceptions.
        }
    }

    // Optional:  Delete all global objects allocated by libprotobuf.
    google::protobuf::ShutdownProtobufLibrary();
}