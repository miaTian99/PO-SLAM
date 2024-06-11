/**
 * Description : Redis + orbslam3_单目执行程序脚本
 * Date :  26/06/2023
 * Author : Shengyang Zhang
 * Modified : Mia Tien
**/
#include<sw/redis++/redis++.h>
#include<iostream>
#include<fstream>
#include<unistd.h>
#include<sys/stat.h>
#include"../build/offline_bbox.pb.h"
using namespace std;
using namespace sw::redis;

//接口回调函数
void call_back(std::string channel, std::string msg)
{
    if (channel=="offline_bbox")
    {
        cout << "channel:" << channel << endl;
        offline_bbox::offline_bboxes bboxes;
        bboxes.ParseFromString(msg);
        // 解析数据，默认不接收图像数据 (数据上行/下传才会涉及到图像的传输)
        string timestamp = bboxes.time_sec();
        string seq_name = bboxes.seq_name(); 
        // 存储收到的数据，若文件不存在则创建文件
        string rootdir1 = "/home/ai-i-tianyaolin/workspace/EAO-SLAM-v1/dataset/SMT/" + seq_name;
        string rootdir2 = "/home/ai-i-tianyaolin/workspace/EAO-SLAM-v1/dataset/SMT/" + seq_name + "/matching_bboxes";
        if(access(rootdir1.c_str(), 0)==-1){
            mkdir(rootdir1.c_str(), 0777); 
        }
        if(access(rootdir2.c_str(), 0)==-1){
            mkdir(rootdir2.c_str(), 0777); 
        }
        string file = rootdir2 + "/" + timestamp + ".txt";
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