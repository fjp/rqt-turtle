#include "rqt_turtle/service_caller.h"
#include <QTreeWidgetItem>
#include <QDialog>
#include <QList>
#include <QStringList>

#include <ros/service.h>
#include <ros/master.h>

#include "ui_ServiceCaller.h"



namespace rqt_turtle {

    ServiceCaller::ServiceCaller(QWidget* parent, std::string service_name)
        : m_pUi(new Ui::ServiceCallerWidget)
        , m_pServiceCallerDialog(this)
        , service_name_(service_name)
    {
        // give QObjects reasonable names
        setObjectName("ServiceCaller");

        ROS_INFO("Initialize ServiceCaller for %s", service_name.c_str());

        m_pUi->setupUi(m_pServiceCallerDialog);

        connect(m_pUi->btnCall, SIGNAL(clicked()), this, SLOT(on_btnCall_clicked()));

        createTreeItems(service_name);
    }


    std::string ServiceCaller::exec_cmd(std::string str_cmd) 
    {
        const char* cmd = str_cmd.c_str();
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


    void ServiceCaller::createTreeItems(std::string service_name)
    {
        ROS_INFO("Create Tree Items for service %s", service_name.c_str());

        // Get service type and args
        std::string cmd_type = "rosservice type " + service_name;
        ROS_INFO("cmd_type: %s", cmd_type.c_str());
        std::string service_type = exec_cmd(cmd_type);
        std::string cmd_info = "rosservice info " + service_name;
        std::string service_info = exec_cmd(cmd_info);
        QString qstr_service_info = QString::fromStdString(service_info);
        QString qstr_args = "Args:";
        int idx = qstr_service_info.indexOf(qstr_args);
        // right(n) returns n rightmost characters, not characters after n.
        QString service_args_line = qstr_service_info.right(qstr_service_info.size() - idx - qstr_args.size());
        ROS_INFO("Service args: %s", service_args_line.toStdString().c_str());
        QStringList qstrArgs = service_args_line.split(QRegExp("\\s+"), QString::SkipEmptyParts);

        // https://doc.qt.io/qt-5/qtreewidget.html#details
        QList<QTreeWidgetItem *> items;
        for (int i = 0; i < qstrArgs.size(); ++i)
        {
            QStringList i_args;
            i_args.append(qstrArgs[i]);
            // TODO type
            i_args.append(QString::fromStdString("float32"));
            // TODO init value depending on type
            i_args.append(QString::fromStdString("0.0"));
            QTreeWidgetItem* item = new QTreeWidgetItem(static_cast<QTreeWidget *>(nullptr), i_args);
            item->setFlags(Qt::ItemIsEditable|Qt::ItemIsEnabled);
            items.append(item);
        }
        m_pUi->request_tree_widget->insertTopLevelItems(0, items);
    }

    void ServiceCaller::on_btnCall_clicked()
    {
        ROS_INFO("Call clicked");

        accept();
    }


    QVariantMap ServiceCaller::getRequest()
    {
        //return m_pUi->request_tree_widget;
        //QList<QTreeWidgetItem*> clist = pRequestTreeWidget->findItems("name", Qt::MatchExactly|Qt::MatchRecursive, 0);
        //foreach(QTreeWidgetItem* item, clist)
        //{
        //    ROS_INFO("output: %s", item->text(2).toStdString().c_str());
        //}
    
        //QMap<QString, QVariant> map;
        QVariantMap map;

        // https://doc.qt.io/archives/qt-4.8/qtreewidgetitemiterator.html#details
        QTreeWidgetItemIterator it(m_pUi->request_tree_widget);
        while (*it)
        {
            QString variable = (*it)->text(0);
            map[variable] = (*it)->text(2);
            ++it;
        }
        return map;
    }

    QString ServiceCaller::getTurtleName()
    {
        return QString("Turtle2");
    }

} // namespace