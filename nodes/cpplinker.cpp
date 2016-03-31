
// Software License Agreement (BSD License)

// Copyright (c) 2016, Matthew Buckley
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// author: Matthew Buckley

#include <sstream>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
//#include <gazebo/World.hh>
#include <string>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <stdlib.h>
#include <set>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//#include <ar_track_alvar/ParamsConfig.h>
#include <gazebo_msgs/LinkStates.h>
const std::string output_frame = "base_footprint";

using namespace std;

//std::set<std::string> * to_watch = NULL;

ros::Subscriber sim_sub_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;

std::string getTFName(std::string & objname,std::string & model) {
    string linkname,namespaced;
    stringstream stream(objname);
    getline(stream, model, ':');
    getline(stream, linkname);
    linkname.erase(0,1);
    namespaced = model + "/" + linkname;
    if (tf_listener->frameExists(namespaced)) {
        return namespaced;
    }
    else if (tf_listener->frameExists(linkname)) {
        return linkname;
    }
    else {
        return "";
    }
}

void callback (const gazebo_msgs::LinkStates & model_msg) {
    int arrsize = model_msg.name.size();
    std::string tfname,objname,parentname;
    /*if (to_watch == NULL) { //get necessary frames
        to_watch = new std::set<std::string>;
        for (int iter=0;iter<arrsize;++iter) {
            objname = string(model_msg.name[iter]);
            tfname = getTFName(objname);
            if (tfname == "") continue;
            std::string temp;
            if (!tf_listener->getParent(tfname,ros::Time(0),temp)) { //I like this a lot more
                to_watch->insert(tfname);
            }
        }
        ROS_INFO("%ld %s",to_watch->size()," disjoint objects found");
    }*/
    tf::Transform mainTrans;
    std::vector<tf::Transform> otherTrans;
    std::vector<std::string> otherTransName;
    bool found = false;
    ros::Time now = ros::Time::now();
    for (int iter=0;iter<arrsize;++iter) {
        objname = string(model_msg.name[iter]);
        std::string modelname;
        tfname = getTFName(objname,modelname);
        if (tfname == "") continue;
        if (tfname == output_frame) {
            tf::Transform posetrans;
            poseMsgToTF(model_msg.pose[iter],posetrans);
            mainTrans = posetrans.inverse();
            found = true;
            continue;
        }
        if((tf_listener->getParent(tfname,ros::Time(0),parentname) && (parentname != output_frame)) || (modelname=="pr2")) continue;
        tf::Transform posetrans;
        poseMsgToTF(model_msg.pose[iter],posetrans);
        otherTrans.push_back(posetrans);
        otherTransName.push_back(tfname);
    }
    tf::Transform temp;
    if(found) {
        for (int iter = 0;iter<otherTrans.size();++iter) {
            temp = mainTrans*otherTrans[iter];
            tf::StampedTransform toPublish(temp,now,output_frame,otherTransName[iter]);
            tf_broadcaster->sendTransform(toPublish);
        }
    }
}


int main(int argc, char *argv[])
{
	ros::init (argc, argv, "tf_linkercpp_node");
	ros::NodeHandle n, pn("~");

	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	
    ros::Duration(0.5).sleep();
    ros::spinOnce();	

    sim_sub_ = n.subscribe ("/gazebo/link_states", 1, &callback);


  ros::Rate rate(60);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    delete tf_listener;
    delete tf_broadcaster;
    return 0;
}
