#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KLGRY "\x1b[90m"
#define KLRED "\x1b[91m"
#define KLGRN "\x1b[92m"
#define KLYEL "\x1b[93m"
#define KLBLU "\x1b[94m"
#define KLMAG "\x1b[95m"
#define KLCYN "\x1b[96m"
#define KLWHT "\x1b[97m"


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

struct PARAM
{
  std::string campose_topic_name;
  std::string outpose_topic_name;
  std::string IF_frame;
  std::string Body_frame;
  std::string drone_frame_id;
  bool use_tf;
  bool debug;
  bool test;
  bool send_tf;
  double delay; // in sec
  double hovering_gain;
  double cam_gain;
  double test_omega;
  void init(ros::NodeHandle& np)
  {
//    ros::NodeHandle np("~");
    //Default values
    campose_topic_name = "/uav_pose_from_cam";
    outpose_topic_name = "/uav_pose_filtered";
    IF_frame = "/map";
    Body_frame = "/cam";
    use_tf = false;
    delay = 0.1;
    hovering_gain = 5.0;
    cam_gain = 5.0;
    drone_frame_id = "/uav";
    debug = false;
    test = true;
    test_omega = 0.4;
    send_tf = false;

    np.getParam("campose_topic_name",campose_topic_name);
    ROS_INFO_STREAM("campose_topic_name: " << campose_topic_name);
    np.getParam("output_pose_topic_name",outpose_topic_name);
    ROS_INFO_STREAM("output_pose_topic_name: " << outpose_topic_name);
    np.getParam("Inertial_frame_id",IF_frame);
    ROS_INFO_STREAM("Inertial_frame_id: " << IF_frame);
    np.getParam("Cam_frame_id",Body_frame);
    ROS_INFO_STREAM("Cam_frame_id: " << Body_frame);
    np.getParam("use_tf_for_cam_pose",use_tf);
    ROS_INFO_STREAM("use_tf_for_cam_pose: " << use_tf);
    np.getParam("total_delay",delay);
    ROS_INFO_STREAM("total_delay: " << delay << " seconds");
    np.getParam("hovering_gain",hovering_gain);
    ROS_INFO_STREAM("hovering_gain: " << hovering_gain);
    np.getParam("send_tf",send_tf);
    if(send_tf) ROS_INFO_STREAM("sending TF  /" << Body_frame << "  to  /uav_filtered");
    np.getParam("cam_gain",cam_gain);
    ROS_INFO_STREAM("cam_gain: " << cam_gain);
    np.getParam("test",test);
    if(test)
    {
      np.getParam("test_omega",test_omega);
      ROS_WARN_STREAM("Test mode, a fake circle transform will be published to test the code angular speed: "
                      << test_omega);

    }
    np.getParam("debug",debug);
    ROS_WARN_COND(debug,"Log mode: debug!");
    if(use_tf)
    {
      np.getParam("drone_frame_id",drone_frame_id);
      ROS_INFO_STREAM("drone_frame_id: " << drone_frame_id);
    }
  }
} params;

class DroneFilter
{
 public:

  DroneFilter(ros::NodeHandle& node)
  {
     ros::NodeHandle n("~");
     params.init(n);
     nh_ = node;
     //Subscribers
     subFromcam_ = nh_.subscribe(params.campose_topic_name, 1, &DroneFilter::camcb, this);

     //Publishers
     IF_pose_publisher = nh_.advertise<geometry_msgs::Pose>(params.outpose_topic_name,1);

     init_att();

  }

  void init_att() //initialize the class attributes
  {
    pose_from_cam.new_msg = false;
    first_data_received = false;
    rate = 200.0;

  }

  void camcb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO_COND(params.debug,"new pose received");
    pose_from_cam.update(*msg);
    if(!first_data_received) {
      ROS_INFO("Cam Measurment Received!");
      first_data_received = true;
    }
  }

  void test_circle_tf(tf::TransformBroadcaster& br,double theta)
  {
    double r = 1.5;

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = params.IF_frame;
    trans.child_frame_id = params.Body_frame;
    trans.transform.translation.x = r*cos(theta);
    trans.transform.translation.y = r*sin(theta);
    trans.transform.translation.z = 0;
    trans.transform.rotation.w = 1;
    br.sendTransform(trans);
//    tf::Transform trans;
//    trans.setOrigin(tf::Vector3(r*cos(theta),r*sin(theta),0.0));
//    trans.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
//    br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),params.IF_frame,params.Body_frame));
  }

  void send_tf(tf::TransformBroadcaster& br,geometry_msgs::Pose msg)
  {
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = params.IF_frame;
    trans.child_frame_id = "/uav_filtered";
    trans.transform.translation.x = msg.position.x;
    trans.transform.translation.y = msg.position.y;
    trans.transform.translation.z = msg.position.z;
    trans.transform.rotation.w = msg.orientation.w;
    trans.transform.rotation.x = msg.orientation.x;
    trans.transform.rotation.y = msg.orientation.y;
    trans.transform.rotation.z = msg.orientation.z;
    br.sendTransform(trans);
  }

  void handle()
  {

     ros::Rate r(rate);
     tf::TransformListener IF_Body_Listener,  //Inertial Frame to Body Frame
                           cam_goal_Listener;
     tf::StampedTransform  IF_Body_Trans,
                           cam_goal_Trans;
     tf::TransformBroadcaster br;
     IF_Body_Trans.setIdentity(); // initializing
     if(!params.test)
       if(!IF_Body_Listener.waitForTransform(params.Body_frame,params.IF_frame,ros::Time(0),ros::Duration(10,0)))
       {
         ROS_FATAL_STREAM("Transform from " << params.IF_frame << " to " << params.Body_frame << " not found!");
         exit(0);
       }
     double th = 0.0;
     first_loop = true;

     while(nh_.ok())
     {
       if(params.test)
       {
//         ROS_INFO_THROTTLE(0.5," <<<<<<< TEST >>>>>>>");
         test_circle_tf(br,th);
         th += params.test_omega/rate;
       }

       if(params.use_tf)
       {
          ROS_INFO_ONCE("getting cam_drone relative pose using TF");
          try
          {
            cam_goal_Listener.lookupTransform(params.Body_frame,params.drone_frame_id,ros::Time(0),cam_goal_Trans);
            ROS_INFO_COND(params.debug,"new pose received");
            pose_from_cam.update(cam_goal_Trans);

            if(!first_data_received) {
              ROS_INFO("Cam Measurment Received!");
              first_data_received = true;
            }
          }
          catch(tf::TransformException ex)
          {
            ROS_WARN("%s",ex.what());
//            ros::Duration(0.3).sleep();
          }
       }

       if(first_data_received)
       {
         if(pose_from_cam.new_msg)
         {
           try
           {
             IF_Body_Listener.lookupTransform(params.IF_frame,params.Body_frame,/*ros::Time(0)*/pose_from_cam.header.stamp - ros::Duration( params.delay),IF_Body_Trans);
             pose_from_cam.pose_IF = IF_Body_Trans * pose_from_cam.pose_body;
             pose_from_cam.new_msg = false;
             /* CAM UPDATE */
             if(!first_loop)
               drone_filtered = (params.hovering_gain * drone_0 + params.cam_gain * pose_from_cam.pose_IF.getOrigin()) *
                   (1/(params.hovering_gain + params.cam_gain));
             else
             {
               first_loop = false;
               drone_filtered = pose_from_cam.pose_IF.getOrigin();
             }
             drone_0 = drone_filtered;
           }
           catch(tf::TransformException ex)
           {
             ROS_WARN_COND(params.debug,"%s",ex.what());
//             ros::Duration(0.3).sleep();
           }
         }
//         else
//           drone_filtered = drone_0;
         geometry_msgs::Pose msg;
         msg.position.x = drone_filtered.getX();
         msg.position.y = drone_filtered.getY();
         msg.position.z = drone_filtered.getZ();
         msg.orientation.w = 1.0;  //orientation is not being considered here

         IF_pose_publisher.publish(msg);
         if(params.send_tf) send_tf(br,msg);
       }
       else if(!params.use_tf)
         ROS_WARN_THROTTLE(1,"Still waiting for the cam measurment");


       r.sleep();
       ros::spinOnce();

     }
  }

protected:
  ros::NodeHandle nh_;
  // Subscribers
  ros::Subscriber subFromcam_;
  // Publishers
  ros::Publisher IF_pose_publisher;
  double rate;
  bool first_loop;
private:
  struct TFPOSESTAMPED{
    tf::Pose pose_body;
    tf::Pose pose_IF;
    std_msgs::Header header;
    bool new_msg;
    void update(geometry_msgs::PoseStamped msg)
    {
      new_msg = true;
      tf::poseMsgToTF(msg.pose,pose_body);
      header = msg.header;
    }
    void update(tf::StampedTransform trans)
    {
      new_msg = true;
      pose_body.setOrigin(trans.getOrigin());
      pose_body.setRotation(trans.getRotation());
      header.stamp = trans.stamp_;
//      ROS_INFO_STREAM(KLGRN<<trans.stamp_);
    }
  }pose_from_cam;
  tf::Vector3 drone_filtered,drone_imu,drone_0;
  bool first_data_received;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_pose_node");
  ros::NodeHandle n;
  DroneFilter node(n);
  node.handle();
  return 0;
}
