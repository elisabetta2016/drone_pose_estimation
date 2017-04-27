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
  double delay; // in sec
  double hovering_gain;
  double cam_gain;
  void init()
  {
    ros::NodeHandle np("~");
    campose_topic_name = "/uav_pose_from_cam";
    outpose_topic_name = "/uav_pose_filtered";
    IF_frame = "/map";
    Body_frame = "/cam";
    use_tf = false;
    delay = 0.1;
    hovering_gain = 5.0;
    cam_gain = 5.0;
    drone_frame_id = "/uav";
    np.param("campose_topic_name",campose_topic_name);
    ROS_INFO_STREAM("campose_topic_name: " << campose_topic_name);
    np.param("output_pose_topic_name",outpose_topic_name);
    ROS_INFO_STREAM("output_pose_topic_name: " << outpose_topic_name);
    np.param("Inertial_frame_id",IF_frame);
    ROS_INFO_STREAM("Inertial_frame_id: " << IF_frame);
    np.param("Cam_frame_id",Body_frame);
    ROS_INFO_STREAM("Cam_frame_id: " << Body_frame);
    np.param("use_tf_for_cam_pose",use_tf);
    ROS_INFO_STREAM("use_tf_for_cam_pose: " << use_tf);
    np.param("total_delay",delay,0.1);
    ROS_INFO_STREAM("total_delay: " << delay << " seconds");
    np.param("hovering_gain",hovering_gain);
    ROS_INFO_STREAM("hovering_gain: " << hovering_gain);
    np.param("cam_gain",cam_gain);
    ROS_INFO_STREAM("cam_gain: " << cam_gain);
    if(use_tf)
    {
      np.param("drone_frame_id",drone_frame_id);
      ROS_INFO_STREAM("drone_frame_id: " << drone_frame_id);
    }
  }
} params;

class DroneFilter
{
 public:

  DroneFilter(ros::NodeHandle& node)
  {
     nh_ = node;
     //Subscribers
     subFromcam_ = nh_.subscribe(params.campose_topic_name, 1, &DroneFilter::camcb, this);

     //Publishers
     IF_pose_publisher = nh_.advertise<geometry_msgs::Pose>(params.outpose_topic_name,1);

     init_att();
     params.init();
  }

  void init_att() //initialize the class attributes
  {
    pose_from_cam.new_msg = false;
    first_data_received = false;
    rate = 10.0;
  }

  void camcb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ROS_INFO("new pose received");
    pose_from_cam.update(*msg);
    if(!first_data_received) {
      ROS_INFO("Cam Measurment Received!");
      first_data_received = true;
    }
  }

  void handle()
  {

     ros::Rate r(rate);
     tf::TransformListener Body_IF_Listener,  //Inertial Frame to Body Frame
                           cam_goal_Listener;
     tf::StampedTransform  Body_IF_Trans,
                           cam_goal_Trans;
     Body_IF_Trans.setIdentity(); // initializing
     if(!Body_IF_Listener.waitForTransform(params.Body_frame,params.IF_frame,ros::Time(0),ros::Duration(10,0)))
     {
       ROS_FATAL_STREAM("Transform from " << params.IF_frame << " to " << params.Body_frame << " not found!");
       exit(0);
     }

     while(nh_.ok())
     {
       if(first_data_received)
       {
         if(pose_from_cam.new_msg)
         {
           try
           {
             Body_IF_Listener.lookupTransform(params.Body_frame,params.IF_frame,pose_from_cam.header.stamp - ros::Duration(params.delay),Body_IF_Trans);
             pose_from_cam.pose_IF = Body_IF_Trans * pose_from_cam.pose_body;
             pose_from_cam.new_msg = false;
             /* CAM UPDATE */
             drone_filtered = (params.hovering_gain * drone_0 + params.cam_gain * pose_from_cam.pose_IF.getOrigin()) *
                 (1/(params.hovering_gain + params.cam_gain));

             drone_0 = drone_filtered;
           }
           catch(tf::TransformException ex)
           {
             ROS_WARN("%s",ex.what());
             ros::Duration(0.3).sleep();
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
       }
       else if(params.use_tf)
       {
          ROS_INFO_ONCE("getting cam_drone relative pose using TF");
          try
          {
            cam_goal_Listener.lookupTransform(params.Body_frame,params.drone_frame_id,ros::Time(0),cam_goal_Trans);
            ROS_INFO("new pose received");
            pose_from_cam.update(cam_goal_Trans);

            if(!first_data_received) {
              ROS_INFO("Cam Measurment Received!");
              first_data_received = true;
            }
          }
          catch(tf::TransformException ex)
          {
            ROS_WARN("%s",ex.what());
            ros::Duration(0.3).sleep();
          }
       }
       else
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
      pose_body.setOrigin(trans.getOrigin());
      pose_body.setRotation(trans.getRotation());
    }
  }pose_from_cam;
  tf::Vector3 drone_filtered,drone_imu,drone_0;
  bool first_data_received;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_pose_node");
  ros::NodeHandle node;

//  tester tt(node);
//  tt.handle();
  return 0;
}
