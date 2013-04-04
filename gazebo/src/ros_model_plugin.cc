#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "math/Vector3.hh"
#include <math/gzmath.hh>

static float Vlin=0.0, Vang=0.0; // set as global 

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {   
    public: ROSModelPlugin()
    {
      // Start up ROS
      std::string name1 = "gazebo"; // this is what appears in the rostopics
      int argc = 0;
      ros::init(argc, NULL, name1);      
    }
    public: ~ROSModelPlugin()
    {
      delete this->nh;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // ROS Nodehandle
      this->nh = new ros::NodeHandle("~");
      // ROS Subscriber
      this->subvel = this->nh->subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &ROSModelPlugin::ROSCallback_Vel, this );

      this->nh = new ros::NodeHandle("~");
      this->pubscan = this->nh->advertise<sensor_msgs::LaserScan>("/scan",1000);

      this->nh = new ros::NodeHandle("~");
      this->pubodom = this->nh->advertise<nav_msgs::Odometry>("/odom",1000);

//***********************************************
      sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor("laser");
      /*if(!sensor)
        printf("sensor is NULL\n");
      this->raysensor = boost::shared_dynamic_cast<sensors::RaySensor>(sensor);
      if(!this->raysensor)
        printf("raysensor is NULL\n");
        */
//***********************************************

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      // for gazebo 1.4 or lower it should be ConnectWorldUpdateStart
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      std::vector<double> rangesgz;
            static double qw,qx,qy,qz, Rrad, Prad, Yrad;

// ************* QUATERNION / POSE DATA ******************
      math::Vector3 p = model->GetWorldPose().pos;
      math::Quaternion r = model->GetWorldPose().rot;
      //from quaternion to Roll Pitch Yaw, in radianos
      qw=r.w;   qx=r.x;     qy=r.y;     qz=r.z;
      Rrad=atan2(  2*(qw*qx+qy*qz),  1-2*(qx*qx+qy*qy)  );  //Roll
      Prad=asin(2*(qw*qy-qz*qx));               //Pitch
      Yrad=atan2(  2*(qw*qz+qx*qy),  1-2*(qy*qy+qz*qz)  );  //Yaw

// ***************************************************
            ros::Time current_time, last_time;
            current_time = ros::Time::now();
            last_time = ros::Time::now();

// *********** SCAN DATA *****************************
            //Pub Scan msg
            /*
            sensor_msgs::LaserScan scan2ros;
            scan2ros.header.stamp=current_time; //ros::Time::now();
            scan2ros.header.frame_id="base_scan";
            scan2ros.angle_min=this->raysensor->GetAngleMin().Radian();
            scan2ros.angle_max=this->raysensor->GetAngleMax().Radian();
            scan2ros.angle_increment=this->raysensor->GetAngleResolution();
            scan2ros.time_increment=0.0;
            scan2ros.scan_time=0.0;
            scan2ros.range_min=this->raysensor->GetRangeMin();
            float rmn=scan2ros.range_min;
            scan2ros.range_max=this->raysensor->GetRangeMax();
            float rmx=scan2ros.range_max;
            // *********************************************
            this->raysensor->GetRanges(rangesgz);
            int raynumber=this->raysensor->GetRangeCount();
            scan2ros.ranges.resize(raynumber);
            for (int iray=0;iray<raynumber;iray++)
            {
                // i found that for some unknown reason the laser scan subtracts,
                // to each laser measure, the min range defined on the laser model
                // to "correct" this when building the scan message for ROS i add
                // that valor if the sum is not greater than max range
                float rg = this->raysensor->GetRange(iray);
                if(rg+rmn<=rmx)
                    {scan2ros.ranges[iray]=rg+rmn;}
                else {scan2ros.ranges[iray]=rmx;}
            }
            // *********************************************
            this->pubscan.publish(scan2ros);
*/
// ******************************************
            //Pub Pose msg
            geometry_msgs::Point p2ros;
            p2ros.x=p.x; p2ros.y=p.y; p2ros.z=p.z;

            geometry_msgs::Quaternion r2ros;
            r2ros.x=r.x; r2ros.y=r.y; r2ros.z=r.z; r2ros.w=r.w;

            nav_msgs::Odometry odom2ros;
        //    odom2ros.header.stamp=scan2ros.header.stamp;//ros::Time::now();
            odom2ros.header.frame_id="base_link";
            odom2ros.pose.pose.position=p2ros;
            odom2ros.pose.pose.orientation=r2ros;
            //odom2ros.pose.covariance= //covariance 6x6 matrix
            
            //Pub velocities
            math::Vector3 angularVelocity = this->model->GetRelativeAngularVel();
            math::Vector3 linearVelocity = this->model->GetRelativeLinearVel();
            
            double linearVel=0;
            if (cos(Yrad) == 0) {
				linearVel=linearVelocity.y/sin(Yrad);
			}
			else {
				linearVel=linearVelocity.x/cos(Yrad);
			}
            //odom2ros.twist.covariance=//covariance 6x6 matrix
            odom2ros.twist.twist.linear.x=linearVel;
            odom2ros.twist.twist.angular.z=angularVelocity.z;

            this->pubodom.publish(odom2ros);

// *********************************************** 
            //set velocities
            Vlin = std::min(Vlin, (float)0.33);
            
            
            float velx,vely;
            velx=Vlin*cos(Yrad);
            vely=Vlin*sin(Yrad);    
            this->model->SetLinearVel(math::Vector3(velx, vely, 0));
            this->model->SetAngularVel(math::Vector3(0, 0, Vang));
            
            
            
            
        

// ***********************************************
            ros::spinOnce();
    }

    // callback functions run every time data is published to the topic
    void ROSCallback_Vel(const geometry_msgs::Twist::ConstPtr& msg)
    {
            Vlin=msg->linear.x;
            Vang=msg->angular.z;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    private: sensors::RaySensorPtr raysensor;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* nh;
    // ROS Subscriber
    ros::Subscriber subvel;

        // ROS Publisher
        ros::Publisher pubscan;
        ros::Publisher pubodom;
        //*************

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
