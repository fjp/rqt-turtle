#ifndef rqt_turtle__action_worker_H
#define rqt_turtle__action_worker_H


#include <QObject>
#include <QRunnable>
#include <QThreadPool>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_actionlib/ShapeAction.h>

#include <rqt_turtle/turtle.h>


namespace rqt_turtle {

    class ActionWorkerKilledException{};


    class ActionWorker : public QObject, public QRunnable
    {
        Q_OBJECT

        actionlib::SimpleActionClient<turtle_actionlib::ShapeAction>& ac_;
        int edges_;
        float radius_;

        bool is_killed_;

    public:
        ActionWorker(actionlib::SimpleActionClient<turtle_actionlib::ShapeAction>& ac, int edges, float radius);


        void run() override;

    signals:
        void progress(int value);

    public slots:
        void kill();
    };

}

#endif // rqt_turtle__action_worker_H