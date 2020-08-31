#include "rqt_turtle/service_caller.h"
#include <QListWidgetItem>
#include <QDialog>

#include <ros/service.h>
#include <ros/master.h>

#include "ui_ServiceCaller.h"



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

    void ServiceCaller::on_btnCall_clicked()
    {
        ROS_INFO("Call clicked.");


        XmlRpc::XmlRpcValue req = "/spawn";
        XmlRpc::XmlRpcValue res;
        XmlRpc::XmlRpcValue pay;

        ros::master::execute("getSystemState", req, res, pay, true);
        ROS_INFO("Size %i", res.size());
        std::string state[res.size()];
        for(int x=0; x < res[2][2].size(); x++)
        {
            std::string gh = res[2][2][x][0].toXml().c_str();
            ROS_INFO(gh.c_str());
            //gh.erase(gh.begin(), gh.begin()+7);
            //gh.erase(gh.end()-8, gh.end());
            //state[x] = gh;
        }

        accept();
    }

    QString ServiceCaller::getTurtleName()
    {
        return QString("Turtle2");
    }

} // namespace