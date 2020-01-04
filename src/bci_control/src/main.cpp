#include "BCI_Mainwindow.h"
#include <QIcon>
#include <robot2BCI.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QString myDir=QCoreApplication::applicationDirPath();
    QDir::setCurrent(myDir);
    a.setWindowIcon(QIcon(":/app.png"));

    //Initialize ROS
    ros::init(argc,argv,"bci_control_node");
    ros::Time::init();
    ROS_INFO("start bci_control process");

    nubot::Robot2BCI robot2BCI;
    MainWindow w(robot2BCI.robot2BCI_info,robot2BCI.BCI2robot_info,robot2BCI.BCI_signal_info);
    w.show();

    robot2BCI.start();
    return a.exec();
}
