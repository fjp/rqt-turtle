#include "rqt_turtle/service_caller.h"
#include <QListWidgetItem>
#include <QDialog>

#include <ros/service.h>
#include <ros/master.h>

#include "ui_ServiceCaller.h"
#include "rqt_turtle/dump_to_stdout.h"



namespace rqt_turtle {

    ServiceCaller::ServiceCaller(QWidget* parent)
        : m_pUi(new Ui::ServiceCallerWidget)
        , m_pServiceCallerDialog(this)
    {
        // give QObjects reasonable names
        setObjectName("ServiceCaller");

        ROS_INFO("INIT");

        m_pUi->setupUi(m_pServiceCallerDialog);


        connect(m_pUi->btnCall, SIGNAL(clicked()), this, SLOT(on_btnCall_clicked()));

        

        //m_pServiceCallerDialog->show();
    }


    bool ServiceCaller::lookupService(const std::string &name)//, std::string &serv_host, uint32_t &serv_port)
    {
        XmlRpc::XmlRpcValue args, result, payload;
        args[0] = "/node_name";//this_node::getName();
        args[1] = name;
        if (!ros::master::execute("lookupService", args, result, payload, false))
            return false;
/*
        string serv_uri(payload);
        if (!serv_uri.length()) // shouldn't happen. but let's be sure.
        {
            ROS_ERROR("lookupService: Empty server URI returned from master");

            return false;
        }

        if (!network::splitURI(serv_uri, serv_host, serv_port))
        {
            ROS_ERROR("lookupService: Bad service uri [%s]", serv_uri.c_str());

            return false;
        }
*/
        
        return true;
    }

    std::string ServiceCaller::execute_command(const char* cmd) 
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    void ServiceCaller::on_btnCall_clicked()
    {
        ROS_INFO("Call clicked.");

        std::string command = "rosservice list";
        std::string result = execute_command(command.c_str());

        lookupService("/spawn");


        XmlRpc::XmlRpcValue request = "/node";
        XmlRpc::XmlRpcValue response;
        XmlRpc::XmlRpcValue payload;

        // http://wiki.ros.org/ROS/Master_API
        ros::master::execute("getSystemState", request, response, payload, true);
        const int num_services = response[2][2].size();
        ROS_INFO("Num services %i", num_services);
        ROS_INFO("%s", response.toXml().c_str());
        std::string service_names[num_services];
        for(int x=0; x < num_services; x++)
        {
            
            std::string service_name = response[2][2][x][0].toXml().c_str();
            
            service_name.erase(service_name.begin(), service_name.begin()+7);
            service_name.erase(service_name.end()-8, service_name.end());
            service_names[x] = service_name;
            ROS_INFO(service_name.c_str());
        }


        ros::master::execute("getTopicTypes", request, response, payload, true);
        const int num_topic_types = response[2].size();
        ROS_INFO("Num topic types %i", num_topic_types);
        ROS_INFO("%s", response.toXml().c_str());

        std::string topic_type_names[num_topic_types];
        for(int i=0; i < num_topic_types; ++i)
        {
            
            std::string topic_type_name = response[2][i][0].toXml().c_str();
            
            topic_type_name.erase(topic_type_name.begin(), topic_type_name.begin()+7);
            topic_type_name.erase(topic_type_name.end()-8, topic_type_name.end());
            topic_type_names[i] = topic_type_name;
            ROS_INFO(topic_type_name.c_str());
        }

        // Call QDialog's accept() method to indicate sucess to the caller
        accept();
    }

    QString ServiceCaller::getTurtleName()
    {
        return QString("Turtle2");
    }

} // namespace