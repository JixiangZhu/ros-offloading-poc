/*
 * dag_nodelet_class.cpp
 *
 * Author: jixiang
 */
// #include <dag_nodelet_class.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <dag/Frame.h>
#include <rosbag/bag.h>
#include <iostream>
#include <fstream>

namespace dag_nodelet_ns
{
class DagNodeletClass : public nodelet::Nodelet 
{
    public:
        DagNodeletClass()
        : value_(0)
        {
            publishers_available = false;
            subscribers_available = false;
            received_ = 0;
            frame_id = 0;
        }

    private:
        double value_;
        std::string nodelet_name_;
        int indegree, outdegree;
        float comp_weight;
        std::vector<std::string> arcs_in_;
        std::map<std::string, float> arcs_out_;
        std::vector<ros::Publisher> pubs;
        std::vector<ros::Subscriber> subs;
        bool publishers_available, subscribers_available;
        ros::NodeHandle nh;
        ros::Timer timer_;
        int received_;
        unsigned frame_id;

        virtual void onInit()
        {
            /* Get parameters
             */
            get_param();
            ROS_INFO_STREAM( nodelet_name_<< ": onInit...");

            /* Setup publishers and subscribers
             */
            set_up();

            ROS_INFO_STREAM(nodelet_name_<<": setup finished");

            /* Run subscribers and publishers
             */
            timer_ = nh.createTimer(ros::Duration(1), boost::bind(&DagNodeletClass::timerCallback, this, _1));
            ROS_INFO_STREAM("==========="
                    <<nodelet_name_ <<": Running"<<
                    "=============");
        }

        void timerCallback(const ros::TimerEvent& event)
        {
            run();
        }

        void run()
        {
            //ros::Rate r(10);  // 10 hz
            if(subscribers_available)
            {
                    ros::spinOnce();
            }
            else
            {
                    first_publish();
            }
        }

        void publish(const dag::Frame::ConstPtr& input)
        {
            /*simulate the computation cost*/
           ros::Duration(comp_weight/2.0).sleep();

           dag::FramePtr output(new dag::Frame);
           //output->data = arcs_out_.at(std::to_string(i));
           output->data = input->data;
           output->header.stamp = ros::Time::now();
           output->header.seq = frame_id;
            //ros::Duration(comp_weight/2.0).sleep();
            if(publishers_available)
            {
                int i = 0;
                for (auto it = arcs_out_.begin();it != arcs_out_.end(); ++it)
                {
                    if (!pubs.at(i))
                    {
                        ROS_WARN("Publisher invalid");
                    } else
                    {   output->data = it->second;
                        pubs.at(i).publish(output);
                        ROS_INFO_STREAM(nodelet_name_<<": publish seccessful! Frame sequence number: "<< output->header.seq);
                    }
                    i++;
                }
                frame_id++;
            }
            else
            {
                /*write the data of the end node*/
                /*
                rosbag::Bag bag;
                bag.open("end.bag", rosbag::bagmode::Write);
                bag.write("frame_id", ros::Time::now(), input->header.seq);
                bag.close();
                */ 
                /* Write to txt file*/
                ROS_INFO_STREAM("write to file");
                const std::string filename="/home/ros/offloading/results/end.txt";
                std::ofstream os;
                os.open(filename, std::ios::app);
                os << ros::Time::now()<<" "<<input->header.seq<<"\n";
                os.close();
                ROS_INFO_STREAM("Write successful!");
            }
        }

        void first_publish()
        {
            ros::Duration(comp_weight/2.0).sleep();
            int i = 0;
            for (auto it=arcs_out_.begin(); it!=arcs_out_.end();++it)
            {
                if (!pubs.at(i))
                {
                    ROS_WARN("Publisher invalid");
                } else
                {
                    dag::FramePtr output(new dag::Frame);
                    //output->data = arcs_out_.at(std::to_string(i));
                    output->data = "Hello world!";
                    output->header.stamp = ros::Time::now();
                    output->header.seq = frame_id;
                    ROS_INFO_STREAM(nodelet_name_<<": start to publish: "<<output->data);
                    pubs.at(i).publish(output);
                    ROS_INFO_STREAM(nodelet_name_<<": publish seccessful! Frame sequence number: "<< output->header.seq);
                }
                i++;
            }
                /* Write to bag file */
                /*
                rosbag::Bag bag;
                bag.open("start.bag", rosbag::bagmode::Write);
                bag.write("frame_id", ros::Time::now(), frame_id);
                bag.close();
                */
                /* Write to txt file*/
                ROS_INFO_STREAM("Start to write start.txt");
                std::string filename = "/home/ros/offloading/results/start.txt";
                std::ofstream os;
                os.open(filename, std::ios::app);
                os << ros::Time::now()<<" "<<frame_id<<"\n";
                os.close();
                ROS_INFO_STREAM("Write to start.txt successful!");
                frame_id++;
        }

        void get_param()
        {
            nodelet::V_string my_argv_ = getMyArgv();

            for (int i=0; i<my_argv_.size();++i)
            {
                ROS_INFO_STREAM("my_argv_ is "<<my_argv_.at(i));
            }

            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            private_nh.getParam("nodelet_name", nodelet_name_);
            private_nh.getParam("value", value_);
            private_nh.getParam("arcs_in", arcs_in_);
            private_nh.getParam("arcs_out", arcs_out_);
            private_nh.getParam("indegree", indegree);
            private_nh.getParam("outdegree", outdegree);
            private_nh.getParam("weight", comp_weight);     
            ROS_INFO_STREAM(nodelet_name_<<":Parameters are obtained");
        }

        void set_up()
        {
            if(indegree != 0)
            {
                set_subscribers();
                subscribers_available = true;
            }

            if(outdegree !=0)
            {
                set_publishers();
                publishers_available = true;
            }
        }

        void set_publishers()
        {
            for(auto it = arcs_out_.begin(); it != arcs_out_.end(); ++it)
            {
                ROS_INFO_STREAM(nodelet_name_<<":Outcoming arc ID: " << it->first << ", data size: "<< it->second);
                
                ros::Publisher pub = nh.advertise<dag::Frame>("edge_"+it->first, 10);
                pubs.push_back(pub);
                ROS_INFO_STREAM(nodelet_name_<<":Publisher setup, publish to topic " << "edge_" + it->first);
                
            }
        }

        void set_subscribers()
        {
            for(unsigned i=0; i < indegree; i++) {

                ROS_INFO_STREAM("Incoming arc ID: " << arcs_in_[i]);
                //MySubscriber my_sub;
                //my_subs.push_back(my_sub);
                ros::Subscriber sub = nh.subscribe("edge_"+arcs_in_[i], 10, &DagNodeletClass::callback, this);
                subs.push_back(sub);
            }
        }

    void callback(const dag::Frame::ConstPtr& input)
    {
        if (!input->data.empty())
        {
            //str->data = msg->data;
            received_++;
            if (received_ == indegree)
            {
                ROS_INFO_STREAM(nodelet_name_<<": Received frame, sequence number: "<< input->header.seq);
                publish(input);
                received_ = 0;
            }
        }
    }
};

PLUGINLIB_DECLARE_CLASS(dag_nodelet_ns, DagNodeletClass, dag_nodelet_ns::DagNodeletClass, nodelet::Nodelet);
}
