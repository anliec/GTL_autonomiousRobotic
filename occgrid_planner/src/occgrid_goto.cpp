
#include <vector>
#include <string>
#include <map>
#include <list>
#include <queue>
#include <utility>
#include <time.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "MovementGenerator.h"
#include "TargetMapBuilder.h"


#define LATIS_MOVEMENT
//#define EXPLORATOR
//#define DISPLAY

#define WIN_SIZE 800

const static int UNACCESIBLE_RADIUS = 5; // unit ? see _info.resolution ??

class OccupancyGridPlanner {
protected:
    ros::NodeHandle nh_;
    ros::Subscriber og_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher path_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher finished_pub_;
    image_transport::Publisher targetMapPub_;
    tf::TransformListener listener_;
    geometry_msgs::PoseStampedConstPtr last_goal;

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
    float goal_threshold;

    TargetMapBuilder targetMapBuilder;

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
                        og_(j, i) = UNKNOWN;
                        break;
                    default:
                        og_(j, i) = FREE;
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
#ifdef DISPLAY
            cv::imshow("OccGrid", resized_og);
#endif
        } else {
            // cv::imshow( "OccGrid", cropped_og_ );
#ifdef DISPLAY
            cv::imshow("OccGrid", og_rgb_);
#endif
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

#ifndef EXPLORATOR
    // This is called when a new goal is posted by RViz. We don't use a
    // mutex here, because it can only be called in spinOnce.
    void target_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
        last_goal = geometry_msgs::PoseStampedConstPtr(msg);
#else
    void target_callback(const std_msgs::Empty void_msg) {
#endif
        ros::Time start_time = ros::Time::now();
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped pose;
        if (!ready) {
            ROS_WARN("Ignoring target while the occupancy grid has not been received");
            return;
        }
        og_rgb_marked_ = og_rgb_.clone();

#ifndef EXPLORATOR //get target from message
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
#else //compute target with exploration map
        // this gets the current pose in transform
        listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
#endif
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
#ifdef DISPLAY
        cv::imshow("OccGrid", og_rgb_marked_);
#endif
        if (!isInGrid(start)) {
            ROS_ERROR("Invalid starting point (%.2f %.2f) -> (%d %d)",
                      transform.getOrigin().x(), transform.getOrigin().y(), start.x, start.y);
            return;
        }
        // If the starting point is not FREE thep1.pt.xre is a bug somewhere, but
        // better to check
        if (og_(start) == OCCUPIED) {
            ROS_ERROR("Invalid start point: occupancy = %d", og_(start));
            og_(start) = FREE;
//            return;
        }
        else if(og_(start) == UNKNOWN){
            ROS_INFO("set robot position as free (was unknow)");
            for(int x=-1 ; x<=1 ; x++){
                for(int y=-1 ; y<=1 ; y++){
                    cv::Point p = start;
                    p.x += x;
                    p.y += y;
                    og_(p) = FREE;
                }
            }
        }
#ifdef EXPLORATOR
        std::vector<cv::Point> inaccessiblePoints;
        tf::Quaternion qRobot(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getOrigin().w());
        GoalHeap heap = targetMapBuilder.computeGoals(og_, start, float(qRobot.getAngle()));
    get_new_element_from_heap:
        if(heap.empty()){
            ROS_INFO("Nothing to explore...");
            std_msgs::Bool finished_msg;
            finished_msg.data = 1; // 1 == true
            finished_pub_.publish(finished_msg);
            return;
        }
        cv::Point target = heap.top().second;
        ROS_INFO("Planning target score = %.5f", heap.top().first);
        heap.pop();
        for(cv::Point &p : inaccessiblePoints){
            if(abs(target.x - p.x) >= UNACCESIBLE_RADIUS && abs(target.y - p.y) >= UNACCESIBLE_RADIUS){
                goto get_new_element_from_heap;
            }
        }
#endif
        ROS_INFO("Planning target: %.2f %.2f -> %d %d",
                 pose.pose.position.x, pose.pose.position.y, target.x, target.y);
        cv::circle(og_rgb_marked_, target, 10, cv::Scalar(0, 0, 255));
#ifdef DISPLAY
        cv::imshow("OccGrid", og_rgb_marked_);
#endif
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


        ROS_INFO("Starting planning from (%d, %d) to (%d, %d)", start.x, start.y, target.x, target.y);

#ifndef LATIS_MOVEMENT
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
            tf::Quaternion qAngle;
            qAngle.setRPY(0.0, 0.0, 0.0);
            path.poses[ipose].pose.orientation.x = qAngle.x();
            path.poses[ipose].pose.orientation.y = qAngle.y();
            path.poses[ipose].pose.orientation.z = qAngle.z();
            path.poses[ipose].pose.orientation.w = qAngle.w();
            ipose++;
            it++;
        }
#else
        Pos3D start3D, target3D;
        start3D.pt = start;
        target3D.pt = target;
        tf::Quaternion qStart(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getOrigin().w());
        tf::Quaternion qTarget(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w); //pose is not set...
        start3D.angle =  unsigned(round(qStart.getAngle()  * double(NUMBER_OF_ANGLES_LEVELS) / (2.0 * M_PI)));
        target3D.angle = unsigned(round(qTarget.getAngle() * double(NUMBER_OF_ANGLES_LEVELS) / (2.0 * M_PI)));
        // Here the A* algorithm is run
        std::list<Pos3D> listPath = AStar3D(start3D, target3D);

        if (listPath.empty()) {
            // No path found
#ifdef EXPLORATOR
            ROS_INFO("Path not fund, trying with a new target");
            inaccessiblePoints.push_back(target);
            goto get_new_element_from_heap;
#else
            ROS_ERROR("No path found from (%d, %d) to (%d, %d)", start.x, start.y, target.x, target.y);
            return;
#endif
        }
        ROS_INFO("sending path");
        // Finally finished_pub_create a ROS path message
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = frame_id_;
        path.poses.resize(listPath.size());
        std::list<Pos3D>::const_iterator it = listPath.begin();
        unsigned int ipose = 0;
        while (it != listPath.end()) {
            // time stamp is not updated because we're not creating a
            // trajectory at this stage
            path.poses[ipose].header = path.header;
            cv::Point P = it->pt - og_center_;
            path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
            path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
            tf::Quaternion qAngle;
            qAngle.setRPY(0.0, 0.0, it->angle * 2.0 * M_PI / double(NUMBER_OF_ANGLES_LEVELS));
            path.poses[ipose].pose.orientation.x = qAngle.x();
            path.poses[ipose].pose.orientation.y = qAngle.y();
            path.poses[ipose].pose.orientation.z = qAngle.z();
            path.poses[ipose].pose.orientation.w = qAngle.w();
            ipose++;
            it++;
        }
#endif
#ifdef EXPLORATOR
        cv::Mat targetMap;
        cv::cvtColor(og_.clone(), targetMap, cv::COLOR_GRAY2BGR);
        float bestScore = heap.top().first;
        while(!heap.empty()){
            cv::Point p = heap.top().second;
            uint8_t value = static_cast<uint8_t>(heap.top().first * 255.0f / bestScore);
            targetMap.at<cv::Vec3b>(p) = {0, value, uint8_t(255 - value)};
            heap.pop();
        }
        for(const Pos3D &pos : listPath){
            targetMap.at<cv::Vec3b>(pos.pt) = {255, 0, 0};
        }
        targetMap.at<cv::Vec3b>(start) = {0, 0, 255};
        sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", targetMap).toImageMsg();
        targetMapPub_.publish(imageMessage);
#endif
        path_pub_.publish(path);
        ROS_INFO("Request completed in %f seconds", (ros::Time::now() - start_time).toSec());
    }

public:

#ifndef EXPLORATOR
    void republish_goal(){
        if(last_goal == nullptr){
            return;
        }
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped pose;
        // This converts target in the grid frame.
        listener_.waitForTransform(frame_id_, last_goal->header.frame_id, last_goal->header.stamp, ros::Duration(1.0));
        listener_.transformPose(frame_id_, *last_goal, pose);
        // this gets the current pose in transform
        listener_.lookupTransform(frame_id_, base_link_, ros::Time(0), transform);
        cv::Point target = cv::Point(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution)
                           + og_center_;
        cv::Point start;
        start = cv::Point(transform.getOrigin().x() / info_.resolution,
                          transform.getOrigin().y() / info_.resolution)
                + og_center_;
        if ((abs(start.x - target.x) <=  goal_threshold) &&
                (abs(start.y - target.y) <=  goal_threshold)) {
            ROS_INFO("republishing goal to recompute path");
            goal_pub_.publish(last_goal);
        } else {
            ROS_INFO("goal already achived, skipping republish");
        }
    }
#else
    void republish_goal(){
        goal_pub_.publish(std_msgs::Empty());
        ROS_INFO("msg published");
    }
#endif

    OccupancyGridPlanner() : nh_("~") {
        int nbour = 4;
        ready = false;
        nh_.param("robot_radius", robot_radius, 0.3);
        nh_.param("base_frame", base_link_, std::string("/body"));
        nh_.param("debug", debug, false);
        nh_.param("neighbourhood", nbour, nbour);
        nh_.param("goal_threshold", goal_threshold, 0.1f);
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
#ifndef EXPLORATOR
        target_sub_ = nh_.subscribe("goal", 1, &OccupancyGridPlanner::target_callback, this);
#else
        target_sub_ = nh_.subscribe("explore", 1, &OccupancyGridPlanner::target_callback, this);
#endif
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
#ifndef EXPLORATOR
        goal_pub_ = nh_.advertise<nav_msgs::Path>("goal", 1, true);
#else
        goal_pub_ = nh_.advertise<std_msgs::Empty>("explore", 1, true);
        finished_pub_ = nh_.advertise<std_msgs::Bool>("finished", 1, true);
        image_transport::ImageTransport it(nh_);
        targetMapPub_ = it.advertise("target_map", 1, false);
#endif
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
    std::list<cv::Point> AStar(const cv::Point &start, const cv::Point &target) const{
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
                if (og_(np) != OCCUPIED && dist_array[np.x][np.y] == -1.0f) {
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
     * An heuristic distance from point p1 to point p2 (this heuristic to build around the fact that you can only move
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
            ret.angle = (angle + a.get_da()) % NUMBER_OF_ANGLES_LEVELS; //prevent array out of bound
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

    inline unsigned toLinearCord(const int &x, const int &y, const unsigned &z) const {
        return (x * og_.size[1] + y) * NUMBER_OF_ANGLES_LEVELS + z;
    }

    /**
     * Compute a A* path from start to target using og_ as ocupency grid
     * @param start point where the path will start
     * @param target point where the path will end
     * @return list of neighbors points giving the shortest path from start to target (or an empty list if no path is
     * found).
     */
    std::list<Pos3D> AStar3D(const Pos3D &start, const Pos3D &target) const{
        MovementGenerator movement_generator;
        //set accumulator arrays
        //ROS_INFO("i need %d bit in ram", sizeof(PointState)*og_.size[0] * og_.size[1] * NUMBER_OF_ANGLES_LEVELS);
        PointState* explored = new PointState[og_.size[0] * og_.size[1] * NUMBER_OF_ANGLES_LEVELS];

        //init loop variables
        AStarHeap3D gray;
        addToHeap3D(gray, start, target, 0.0f);
        explored[toLinearCord(start.pt.x, start.pt.y, start.angle)] = PointState(0.0f, start);
//        float shortest_path = std::numeric_limits<float>::max();

        while (!gray.empty()){
//            if(gray.top().first > shortest_path){
//                gray.pop();
//                continue;
//            }
            Pos3D p = gray.top().second;
            gray.pop();
            float cur_dist = explored[toLinearCord(p.pt.x,p.pt.y,p.angle)].dist;

            //check if target found
            if(p == target){
//                shortest_path = cur_dist;
//                continue;
                break;
            }

            std::vector<AngleMovement> possibleMove = movement_generator.getPossibleMove(p.angle);
            for(const AngleMovement &m : possibleMove){
                Pos3D np = p + m;
                if (og_(np.pt) != OCCUPIED && explored[toLinearCord(np.pt.x,np.pt.y,np.angle)].dist == -1.0f) {
                    float dist = cur_dist + m.get_cost();
                    explored[toLinearCord(np.pt.x,np.pt.y,np.angle)] = PointState(dist, p);
                    addToHeap3D(gray, np, target, dist);
                }
            }
        }

        //Now convert the path found into a list of points
        std::list<Pos3D> path;

        //if no path was found, return an empty path
        if(explored[toLinearCord(target.pt.x,target.pt.y,target.angle)].dist == -1.0f){
            //clean memory
            delete [] explored;
            return path;
        }

        const Pos3D * pred = &target;
        while(*pred != start){
            path.push_front(*pred);
            pred = &explored[toLinearCord(pred->pt.x,pred->pt.y,pred->angle)].pred;
        }

        //clean memory
        delete [] explored;

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
        float dist = current_dist + distHeuristic3D(p, target);
        heap.push(HeapElement3D(dist, p));
    }

    static float distHeuristic3D(const Pos3D &p1, const Pos3D &p2){
        float dist;
        cv::Point delta = p2.pt - p1.pt;
        dist = delta.x * delta.x + delta.y * delta.y;
        return sqrtf(dist);
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "occgrid_planner");
    OccupancyGridPlanner ogp;
//    cv::namedWindow("OccGrid", CV_WINDOW_AUTOSIZE);
    ros::Rate loop_rate(0.2);
    while (ros::ok()) {
        ros::spinOnce();
//        if (cv::waitKey(50) == 'q') {
//            ros::shutdown();
//        }
        ogp.republish_goal();
        loop_rate.sleep();
    }
}

