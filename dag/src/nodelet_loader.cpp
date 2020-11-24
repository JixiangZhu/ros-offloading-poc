#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <nodelet/loader.h>
#include <nodelet/NodeletLoad.h>
#include <dag/InitiateNodelets.h>
#include <vector>
#include <unistd.h>

class InitNodelets
{
private:
    dag::InitiateNodelets::Request request;
    bool service_called = false;
    
public:
    bool init_nodelets(dag::InitiateNodelets::Request &req,
            dag::InitiateNodelets::Response &res)
    {
        //nodelet::M_string remap(ros::names::getRemappings());
        this->request = req;
        ROS_INFO_STREAM("req[0].id = "<<req.nodes[0].id);
        size_t size = req.nodes.size();
        
        res.node_initiated = true;
        this->service_called = true;
        ROS_INFO_STREAM("Init Service return with true");
        return true;
    }

    dag::InitiateNodelets::Request get_request()
    {
        return this->request;
    }
    
    bool service_is_called()
    {
        return this->service_called;
    }
};

void load_nodelet(dag::InitiateNodelets::Request &req, nodelet::Loader &loader)
{    
    nodelet::M_string remap(ros::names::getRemappings());
    
    for(int i=0; i<req.nodes.size();i++)
    {
        dag::Node node = req.nodes[i];
        std::string s = "nodelet_id: " + std::to_string(node.id);
        nodelet::V_string nargv;
        nargv.push_back(s);

        std::string nodelet_name("nodelet_" + std::to_string(node.id));
         /* nh1 namespace /node_loader */
        ros::NodeHandle nh1("~");

        /* nh2 namespace /@node_name */
        ros::NodeHandle nh2(nodelet_name);

        /* Build the inbound arcs and outbound arcs map*/
        std::vector<std::string> arcs_in;
        std::map<std::string, float> arcs_out;
        for(int i=0;i<node.incomes.size();i++)
            arcs_in.push_back(std::to_string(node.incomes[i].id));
        
        for(int i=0;i<node.outcomes.size();i++)
            arcs_out[std::to_string(node.outcomes[i].id)] = node.outcomes[i].weight;
        
         /* Build the outbound arcs map between the arc_id and the data size */
        ROS_INFO_STREAM("arcs_out built");
        nh2.setParam("nodelet_name", nodelet_name);
        nh2.setParam("node_weight",node.weight);
        nh2.setParam("indegree", node.indegree);
        nh2.setParam("outdegree", node.outdegree);
        nh2.setParam("arcs_in", arcs_in);
        nh2.setParam("arcs_out", arcs_out);
        ROS_INFO_STREAM("parameters set");
        loader.load(nodelet_name, "dag_nodelet_ns/DagNodeletClass", remap, nargv);
    }

}


int main(int argc, char** argv)
{
    // get hostname
    char hostname[1024];
    gethostname(hostname, 1024);
    std::string host(hostname);
    // std::string node_name(host);
    std::string node_name("nodelet_loader");

    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
    ros::NodeHandle h;
    ros::NodeHandle rh(node_name);
    nodelet::Loader loader(true);
    
    /**
     * Service for initiate the nodelets remotely, providing the nodelet id and the dag information
     */ 
    InitNodelets init;

    ros::ServiceServer service = rh.advertiseService("init_nodelets_service", &InitNodelets::init_nodelets, &init);

    ros::Rate r(10);   
    while(ros::ok() && init.get_request().nodes.size()== 0)
    {
        ros::spinOnce();
        r.sleep();
    }

    dag::InitiateNodelets::Request request = init.get_request();
    load_nodelet(request, loader);
    
    ROS_INFO_STREAM("Ready to initiate some nodelets...");
        

    std::vector<std::string> loaded_nodelets = loader.listLoadedNodelets();
    ROS_INFO_STREAM("Loaded nodelets list: ");
    for (std::vector<std::string>::const_iterator i = loaded_nodelets.begin(); i != loaded_nodelets.end(); ++i)
        ROS_INFO_STREAM(*i);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();    
    return 0;
}
