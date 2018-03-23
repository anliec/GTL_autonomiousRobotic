
#include <vector>
#include <string>
#include <map>
#include <list>
#include <queue>
#include  <utility>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "MovementGenerator.h"

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

class OccupancyGridPlanner {
protected:
    ros::NodeHandle nh_;
    ros::Subscriber og_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher path_pub_;
    tf::TransformListener listener_;

    cv::Rect roi_;
    cv::Mat_<uint8_t> og_, cropped_og_;
    cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
    cv::Point og_center_;
    nav_msgs::MapMetaData info_;
    std::string frame_id_;
    std::string base_link_;
    double robot_radius;
    unsigned int neighbourhood_;
    bool ready;
    bool debug;

    typedef std::multimap<float, cv::Point> Heap;

    // Callback for Occupancy Grids
    void og_callback(const nav_msgs::OccupancyGridConstPtr &msg) {
        info_ = msg->info;
        frame_id_ = msg->header.frame_id;
        // Create an image to store the value of the grid.
        og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width, 0xFF);
        og_center_ = cv::Point(-info_.origin.position.x / info_.resolution,
                               -info_.origin.position.y / info_.resolution);

        // Some variables to select the useful bounding box
        unsigned int maxx = 0, minx = msg->info.width,
                maxy = 0, miny = msg->info.height;
        // Convert the representation into something easy to display.
        for (unsigned int j = 0; j < msg->info.height; j++) {
            for (unsigned int i = 0; i < msg->info.width; i++) {
                int8_t v = msg->data[j * msg->info.width + i];
                switch (v) {
                    case 0:
                        og_(j, i) = FREE;
                        break;
                    case 100:
                        og_(j, i) = OCCUPIED;
                        break;
                    case -1:
                    default:
                        og_(j, i) = UNKNOWN;
                        break;
                }
                // Update the bounding box of free or occupied cells.
                if (og_(j, i) != UNKNOWN) {
                    minx = std::min(minx, i);
                    miny = std::min(miny, j);
                    maxx = std::max(maxx, i);
                    maxy = std::max(maxy, j);
                }
            }
        }
        double erosion_size = robot_radius / info_.resolution;
        cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
                                                 cv::Size( int(2.0*erosion_size + 1.0), int(2.0*erosion_size+1.0) ),
                                                 cv::Point( int(erosion_size), int(erosion_size) ) );
        /// Apply the erosion operation
        erode( og_, og_, element );

        if (!ready) {
            ready = true;
            ROS_INFO("Received occupancy grid, ready to plan");
        }
        // The lines below are only for display
        unsigned int w = maxx - minx;
        unsigned int h = maxy - miny;
        roi_ = cv::Rect(minx, miny, w, h);
        cv::cvtColor(og_, og_rgb_, CV_GRAY2RGB);
        // Compute a sub-image that covers only the useful part of the
        // grid.
        cropped_og_ = cv::Mat_<uint8_t>(og_, roi_);
        if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
            // The occupancy grid is too large to display. We need to scale
            // it first.
            double ratio = w / ((double) h);
            cv::Size new_size;
            if (ratio >= 1) {
                new_size = cv::Size(WIN_SIZE, WIN_SIZE / ratio);
            } else {
                new_size = cv::Size(WIN_SIZE * ratio, WIN_SIZE);
            }
            cv::Mat_<uint8_t> resized_og;
            cv::resize(cropped_og_, resized_og, new_size);
            cv::imshow("OccGrid", resized_og);
        } else {
            // cv::imshow( "OccGrid", cropped_og_ );
            cv::imshow("OccGrid", og_rgb_);
        }
    }

    // Generic test if a point is within the occupancy grid
    bool isInGrid(const cv::Point &P) {
        if ((P.x < 0) || (P.x >= (signed) info_.width)
            || (P.y < 0) || (P.y >= (signed) info_.height)) {
            return false;
        }
        return true;
    }

    double heuristic(const cv::Point &current_point, const cv::Point &goal_point){
        return cv::norm(current_point-goal_point);
    }

    // This is called when a new goal is posted by RViz. We don't use a
    // mutex here, because it can only be called in spinOnce.
    void target_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped pose;
        if (!ready) {
            ROS_WARN("Ignoring target while the occupancy grid has not been received");
            return;
        }
        ROS_INFO("Received planning request");
        og_rgb_marked_ = og_rgb_.clone();
        // Convert the destination point in the occupancy grid frame.
        // The debug case is useful is the map is published without
        // gmapping running (for instance with map_server).
        if (debug) {
            pose = *msg;
        } else {
            // This converts target in the grid frame.
            listener_.waitForTransform(frame_id_, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
            listener_.transformPose(frame_id_, *msg, pose);
            // this gets the current pose in transform
            listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
        }
        // Now scale the target to the grid resolution and shift it to the
        // grid center.
        cv::Point target = cv::Point(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution)
                           + og_center_;
        ROS_INFO("Planning target: %.2f %.2f -> %d %d",
                 pose.pose.position.x, pose.pose.position.y, target.x, target.y);
        cv::circle(og_rgb_marked_, target, 10, cv::Scalar(0, 0, 255));
        cv::imshow("OccGrid", og_rgb_marked_);
        if (!isInGrid(target)) {
            ROS_ERROR("Invalid target point (%.2f %.2f) -> (%d %d)",
                      pose.pose.position.x, pose.pose.position.y, target.x, target.y);
            return;
        }
        // Only accept target which are FREE in the grid (HW, Step 5).
        if (og_(target) != FREE) {
            ROS_ERROR("Invalid target point: occupancy = %d", og_(target));
            return;
        }

        // Now get the current point in grid coordinates.
        cv::Point start;
        if (debug) {
            start = og_center_;
        } else {
            start = cv::Point(transform.getOrigin().x() / info_.resolution,
                              transform.getOrigin().y() / info_.resolution)
                    + og_center_;
        }
        ROS_INFO("Planning origin %.2f %.2f -> %d %d",
                 transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
        cv::circle(og_rgb_marked_, start, 10, cv::Scalar(0, 255, 0));
        cv::imshow("OccGrid", og_rgb_marked_);
        if (!isInGrid(start)) {
            ROS_ERROR("Invalid starting point (%.2f %.2f) -> (%d %d)",
                      transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
            return;
        }
        // If the starting point is not FREE thep1.pt.xre is a bug somewhere, but
        // better to check
        if (og_(start) != FREE) {
            ROS_ERROR("Invalid start point: occupancy = %d", og_(start));
            return;
        }
        ROS_INFO("Starting planning from (%d, %d) to (%d, %d)", start.x, start.y, target.x, target.y);

        // Here the A* algorithm is run
        std::list<cv::Point> listPath = AStar(start, target);

        if (listPath.size() == 0) {
            // No path found
            ROS_ERROR("No path found from (%d, %d) to (%d, %d)", start.x, start.y, target.x, target.y);
            return;
        }
        ROS_INFO("Planning completed");
        // Finally create a ROS path message
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = frame_id_;
        path.poses.resize(listPath.size());
        std::list<cv::Point>::const_iterator it = listPath.begin();
        unsigned int ipose = 0;
        while (it != listPath.end()) {
            // time stamp is not updated because we're not creating a
            // trajectory at this stage
            path.poses[ipose].header = path.header;
            cv::Point P = *it - og_center_;
            path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
            path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
            ipose++;
            it++;
        }
        path_pub_.publish(path);
        ROS_INFO("Request completed");
    }


public:
    OccupancyGridPlanner() : nh_("~") {
        int nbour = 4;
        ready = false;
        nh_.param("robot_radius", robot_radius, 0.3);
        nh_.param("base_frame", base_link_, std::string("/body"));
        nh_.param("debug", debug, false);
        nh_.param("neighbourhood", nbour, nbour);
        switch (nbour) {
            case 4:
                neighbourhood_ = nbour;
                break;
            case 8:
                neighbourhood_ = nbour;
                break;
            default:
                ROS_WARN("Invalid neighbourhood specification (%d instead of 4 or 8)", nbour);
                neighbourhood_ = 8;
        }
        og_sub_ = nh_.subscribe("occ_grid", 1, &OccupancyGridPlanner::og_callback, this);
        target_sub_ = nh_.subscribe("goal", 1, &OccupancyGridPlanner::target_callback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
    }

private:
    typedef std::pair<float, cv::Point> HeapElement;

    struct HeapElementCompare{
        bool operator()(HeapElement const &left, HeapElement const &right){
            return left.first > right.first;
        }
    };

    typedef std::priority_queue<HeapElement, std::vector<HeapElement>, HeapElementCompare> AStarHeap;

    /**
     * Compute a A* path from start to target using og_ as ocupency grid
     * @param start point where the path will start
     * @param target point where the path will end
     * @return list of neighbors points giving the shortest path from start to target (or an empty list if no path is
     * found).
     */
    std::list<cv::Point> AStar(const cv::Point &start, const cv::Point &target){
        //set list of explorable neighbors
        cv::Point neighbours[8] = {cv::Point(1, 0), cv::Point(0, 1), cv::Point(-1, 0), cv::Point(0, -1),
                                   cv::Point(1, 1), cv::Point(-1, 1), cv::Point(-1, -1), cv::Point(1, -1)};
        float cost[8] = {1, 1, 1, 1, sqrtf(2), sqrtf(2), sqrtf(2), sqrtf(2)};
        //set accumulator arrays
        float dist_array[og_.size[0]][og_.size[1]];
        cv::Point pred_array[og_.size[0]][og_.size[1]];
        for(int x=0 ; x < og_.size[0] ; x++){
            for(int y=0 ; y < og_.size[1] ; y++){
                dist_array[x][y] = -1.0f;
            }
        }

        //init loop variables
        AStarHeap gray;
        addToHeap(gray, start, target, 0.0f);
        dist_array[start.x][start.y] = 0;
        pred_array[start.x][start.y] = start;

        while (!gray.empty()){
            cv::Point p = gray.top().second;
            gray.pop();

            //check if target found
            if(p == target){
                break;
            }

            float cur_dist = dist_array[p.x][p.y];
            for(unsigned n=0 ; n < 8 ; n++) {
                cv::Point np = p + neighbours[n];
                // if neighbors is free and was not already seen
                if (og_(np) == FREE && dist_array[np.x][np.y] == -1.0f) {
                    float dist = cur_dist + cost[n];
                    pred_array[np.x][np.y] = p;
                    dist_array[np.x][np.y] = dist;
                    addToHeap(gray, np, target, dist);
                }
            }
        }

        //Now convert the path found into a list of points
        std::list<cv::Point> path;

        //if no path was found, return an empty path
        if(dist_array[target.x][target.y] == -1.0f){
            return path;
        }

        const cv::Point * pred = &target;
        while(*pred != start){
            path.push_front(*pred);
            pred = &pred_array[pred->x][pred->y];
        }

        return path;
    }

    /**
     * An heuristic distance from point p1 to point p2 (this heristic to build around the fact that you can only move
     * to a neighbors point)
     * @param p1
     * @param p2
     * @return shortest possible distance between p1 and p2 if no obstacle
     */
    static float distHeuristic(const cv::Point &p1, const cv::Point &p2){
        float dist;
        cv::Point delta = p2 - p1;
        if(delta.x > delta.y){
            dist = float(delta.y) * sqrtf(2);
            dist += float(delta.x - delta.y);
            return dist;
        }
        else{
            dist = float(delta.x) * sqrtf(2);
            dist += float(delta.y - delta.x);
            return dist;
        }
    }

    /**
     * add the given point to the heap, taking care of computing the heuristic best possible distance to target.
     * @param heap the heap were the point will be added
     * @param p the point to add
     * @param target the path target point
     * @param current_dist the current distance to go from start to p
     */
    static void addToHeap(AStarHeap &heap, const cv::Point &p, const cv::Point &target,
                          const float &current_dist){
        float dist = current_dist + distHeuristic(p, target);
        heap.push(std::pair<float, cv::Point>(dist, p));
    }

    //------------------------------------------------------------------//
    // 3D resolution algorithm                                          //
    //------------------------------------------------------------------//

    struct Pos3D{
        cv::Point pt;
        unsigned angle;
        bool operator==(const Pos3D &o) const {
            return o.pt == pt && o.angle == angle;
        }
        bool operator!=(const Pos3D &o) const {
            return o.pt != pt || o.angle != angle;
        }
        Pos3D operator+(const AngleMovement &a) const{
            Pos3D ret;
            ret.pt = pt;
            ret.pt.x += a.get_dx();
            ret.pt.y += a.get_dy();
            ret.angle = angle + a.get_da();
            return ret;
        }
    };
    struct PointState{
        PointState(const float &d=-1.0f, const Pos3D &p=Pos3D()){
            dist = d;
            pred = p;
        }
        float dist;
        Pos3D pred;
    };
    typedef std::pair<float, Pos3D> HeapElement3D;
    struct HeapElement3DCompare{
        bool operator()(HeapElement3D const &left, HeapElement3D const &right){
            return left.first > right.first;
        }
    };
    typedef std::priority_queue<HeapElement3D, std::vector<HeapElement3D>, HeapElement3DCompare> AStarHeap3D;

    /**
     * Compute a A* path from start to target using og_ as ocupency grid
     * @param start point where the path will start
     * @param target point where the path will end
     * @return list of neighbors points giving the shortest path from start to target (or an empty list if no path is
     * found).
     */
    std::list<Pos3D> AStar3D(const Pos3D &start, const Pos3D &target){
        MovementGenerator movement_generator;
        //set accumulator arrays
        PointState explored[og_.size[0]][og_.size[1]][NUMBER_OF_ANGLES_LEVELS];
        for(int x=0 ; x < og_.size[0] ; x++){
            for(int y=0 ; y < og_.size[1] ; y++){
                for(unsigned a=0 ; a < NUMBER_OF_ANGLES_LEVELS ; a++){
                    explored[x][y][a].dist = -1.0f;
                }
            }
        }

        //init loop variables
        AStarHeap3D gray;
        addToHeap3D(gray, start, target, 0.0f);
        explored[start.pt.x][start.pt.y][start.angle] = PointState(0.0f, start);

        while (!gray.empty()){
            Pos3D p = gray.top().second;
            gray.pop();

            //check if target found
            if(p == target){
                break;
            }

            float cur_dist = explored[p.pt.x][p.pt.y][p.angle].dist;
            std::vector<AngleMovement> possibleMove = movement_generator.getPossibleMove(p.angle);
            for(AngleMovement m : possibleMove){
                Pos3D np = p + m;
                if (og_(np.pt) == FREE && explored[np.pt.x][np.pt.y][np.angle].dist == -1.0f) {
                    float dist = cur_dist + m.get_cost();
                    explored[np.pt.x][np.pt.y][np.angle] = PointState(dist, p);
                    addToHeap3D(gray, np, target, dist);
                }
            }
        }

        //Now convert the path found into a list of points
        std::list<Pos3D> path;

        //if no path was found, return an empty path
        if(explored[target.pt.x][target.pt.y][target.angle].dist == -1.0f){
            return path;
        }

        const Pos3D * pred = &target;
        while(*pred != start){
            path.push_front(*pred);
            pred = &explored[pred->pt.x][pred->pt.y][pred->angle].pred;
        }

        return path;
    }

    /**
     * add the given point to the heap, taking care of computing the heuristic best possible distance to target.
     * @param heap the heap were the point will be added
     * @param p the point to add
     * @param target the path target point
     * @param current_dist the current distance to go from start to p
     */
    static void addToHeap3D(AStarHeap3D &heap, const Pos3D &p, const Pos3D &target,
                          const float &current_dist){
        float dist = current_dist + distheuristic3D(p, target);
        heap.push(HeapElement3D(dist, p));
    }

    static float distheuristic3D(const Pos3D &p1, const Pos3D &p2){
        float dist;
        cv::Point delta = p2.pt - p1.pt;
        dist = delta.x * delta.x + delta.y * delta.y;
        return sqrtf(dist);
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "occgrid_planner");
    OccupancyGridPlanner ogp;
    cv::namedWindow("OccGrid", CV_WINDOW_AUTOSIZE);
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey(50) == 'q') {
            ros::shutdown();
        }
    }
}

