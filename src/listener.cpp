#include "ros/ros.h"
#include "std_msgs/String.h"

// #include <string>       // std::string

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


class LetterWriter{
public:
  int a;
  int ds_list; // Change forma

  ros::NodeHandle node;

  LetterWriter(ros::NodeHandle &node_);

  void load_ds(const std::string& input);

  ~LetterWriter(void);

  void run(void);

private:
  int b;

};

LetterWriter::LetterWriter(ros::NodeHandle &node_){
  node = node_;
  ros::Subscriber sub = node.subscribe("chatter", 1000, chatterCallback);
};

LetterWriter::~LetterWriter(){
  std::cout << "\n\nFinished script. \n\n";
};


void LetterWriter::run(void){
  // std::cout << "running new one, too. \n";
  ros::Rate loop_rate(10);

  // Main loop
  while(ros::ok()){
    std::cout<<"Done only thisonce more \n";

    load_ds("asdf");

    loop_rate.sleep();
    ros::spinOnce();
  }
}

void LetterWriter::load_ds(const std::string& input){
  std::cout << "one more \n";
  std::cout << "Open this file " << input << "\n";
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  LetterWriter letterWriter(nh);
  letterWriter.run();
  
  

  return 0;
}
