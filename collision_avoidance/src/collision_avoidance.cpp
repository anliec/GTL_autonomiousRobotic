
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;

        // This might be useful
        double radius;

        pcl::PointCloud<pcl::PointXYZ> lastpc;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            // TODO: modify desired using the laser point cloud

            return res;
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius",radius,1.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


