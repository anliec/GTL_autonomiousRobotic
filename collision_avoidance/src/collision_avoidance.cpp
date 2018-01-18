
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


const static float STOP_DISTANCE = 25.0;
const static float Y_CHECK_RANGE = 1.0;


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
            if(desired.linear.x <= 0.0){
                //if we move backward, we don't see anything...
                return res;
            } else{
                // We move forward, we need to check for obstacles

                // find minimal x value in the last point cloud
                float minDistance = 1000.0;
                for(pcl::PointXYZ p : lastpc) {
                    if(p.y > -Y_CHECK_RANGE && p.y < Y_CHECK_RANGE && p.x > 0.0) {
                        if (p.x < minDistance) {
                            minDistance = p.x;
                        }
                    }
                }

                if(desired.linear.x > minDistance - STOP_DISTANCE){
                    // if we will go too close of an object, adjust distance to stop
                    // at the right distance
//                    res.linear.x = minDistance - STOP_DISTANCE;
                    res.linear.x = 0.0;
                }
                return res;
            }
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0){
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


