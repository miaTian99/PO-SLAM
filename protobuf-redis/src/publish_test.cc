#include<sw/redis++/redis++.h>
#include<iostream>
#include"../build/offline_bbox.pb.h"
using namespace std;
using namespace sw::redis;

void PromptForBbox(offline_bbox::Bbox* obj_bbox) {
    float bbox_p;
    int id, bbox_x, bbox_y, bbox_w, bbox_h;
    // 连续输入
    cout << "Enter info: e.g.(id, bbox_x,bbox_y, bbox_w, bbox_h, bbox_p)." << endl;
    cin >> id >> bbox_x >> bbox_y >> bbox_w >> bbox_h >> bbox_p;
    obj_bbox->set_obj_id(id); obj_bbox->set_bbox_x(bbox_x); obj_bbox->set_bbox_y(bbox_y); 
    obj_bbox->set_bbox_w(bbox_w); obj_bbox->set_bbox_h(bbox_h); obj_bbox->set_bbox_p(bbox_p);
    cout << "current bbox: " << obj_bbox->bbox_x() << " " << obj_bbox->bbox_y() << " " << obj_bbox->bbox_h() << " " << obj_bbox->bbox_w() << " " << obj_bbox->bbox_p() << endl; 
}


int main()
{
    Redis redis("tcp://127.0.0.1:6379");
    string channel = "offline_bbox";
    cout << "publish channel:" << channel << endl;
    cout << "Begin to publish..." << endl;
    offline_bbox::offline_bboxes bboxes;
    string str, msg, timestamp, seq_name;
    cout << "Enter seq_name: ";
    cin >> seq_name;
    bboxes.set_seq_name(seq_name);
    while(1)
    {   
        cout << "add new record? 'y' to coutinue,'n' to exit." << endl;
        getline(cin, str);
        if(str == "n"){
            cout << "total bbox nums: " << bboxes.bbox_size() << endl;
            msg = bboxes.SerializeAsString();
            redis.publish(channel, msg);
            cout << "-------------" << endl;
            break;
        }else if(str == "y"){
            cout << "Enter timestamp: ";
            cin >> timestamp;
            bboxes.set_time_sec(timestamp);
            PromptForBbox(bboxes.add_bbox());
        }
    }
    return 0;
}