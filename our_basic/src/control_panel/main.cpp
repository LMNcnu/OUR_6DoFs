#include "mainwindow.h"
#include <QApplication>
#include <pthread.h>
//#include "our_basic/Joint.h"

pthread_t tid;
//our_basic::Joint joints;
extern std_msgs::Float32MultiArray joints;

struct arg_holder
{
    int argc;
    char **argv;
};

arg_holder *arg_struct;

void *thread_caller(void *arg)
{
    ros::init(arg_struct->argc, arg_struct->argv, "control_panel");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<std_msgs::Float32MultiArray> ("moveJ", 1000);
    ros::Rate loop_rate(200);//Hz

    while(ros::ok())
    {
      command_pub.publish(joints);

      loop_rate.sleep();
    }

 	return 0;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    arg_struct = (arg_holder *)malloc(sizeof(struct arg_holder));
    arg_struct->argc = argc;
    arg_struct->argv = argv;

    int err = pthread_create(&tid, NULL, thread_caller, NULL);

    return a.exec();
}
