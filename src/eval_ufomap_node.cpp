// UFO
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufo/map/occupancy_map_semantic_color.h>
#include <ufo/map/semantic_labels.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/Labels.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>

// STD
#include <limits>
#include <random>
#include <variant>
#include <vector>
#include <unordered_map>

#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

unsigned int num_clouds = 0;
double elapsed_ufomap(0);
unsigned int num_occupied = 0;
unsigned int num_free = 0;
size_t num_nodes = 0;
double max_time = 0.0;
double min_time = std::numeric_limits<double>::max();
bool insert = true;

ros::Publisher pub_;
ros::Publisher label_marker_pub_;
ros::Publisher label_points_pub_;
std::unordered_map<std::string, ros::Publisher> label_publishers_;

std::variant<std::monostate, ufo::map::OccupancyMap, ufo::map::OccupancyMapSemanticColor>
    ufo_map;
double max_range = -1;
unsigned int depth = 0;
bool simple_ray_casting = true;
unsigned int early_stopping = 0;
bool async = false;
bool publish = true;
int publish_every_x = 1;
bool compress = true;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(boost::shared_ptr<M const> const &msg) { this->signalMessage(msg); }
};

void publishLabelMarkerPos(std::vector<ufo::math::Vector3> const &positions, std::vector<ufo::map::Color> const &colors)
{
    visualization_msgs::MarkerArray markerArr;
    markerArr.markers.resize(positions.size());
    std::cout << "positions size = " << positions.size() << ". Colors = " << colors.size() << '\n';

    for (unsigned int i = 0; i < positions.size(); ++i)
    {
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        markerArr.markers[i].header.frame_id = "map";
        markerArr.markers[i].header.stamp = ros::Time::now();
        markerArr.markers[i].type = visualization_msgs::Marker::CUBE;
        markerArr.markers[i].action = visualization_msgs::Marker::ADD;
        markerArr.markers[i].id = i;
        markerArr.markers[i].pose.position.x = positions[i].x();
        markerArr.markers[i].pose.position.y = positions[i].y();
        markerArr.markers[i].pose.position.z = positions[i].z();
        markerArr.markers[i].pose.orientation.x = 0.0;
        markerArr.markers[i].pose.orientation.y = 0.0;
        markerArr.markers[i].pose.orientation.z = 0.0;
        markerArr.markers[i].pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        markerArr.markers[i].scale.x = 0.08;
        markerArr.markers[i].scale.y = 0.08;
        markerArr.markers[i].scale.z = 0.08;

        // Set the color -- be sure to set alpha to something non-zero!
        markerArr.markers[i].color.r = colors[i].r;
        markerArr.markers[i].color.g = colors[i].g;
        markerArr.markers[i].color.b = colors[i].b;
        markerArr.markers[i].color.a = 0.5;
        markerArr.markers[i].lifetime = ros::Duration();
    }
    // markerArr.he
    label_marker_pub_.publish(markerArr);
}

void publishLabelMarkerPoints(std::vector<ufo::math::Vector3> const &positions, std::vector<ufo::map::Color> const &colors)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.color.a = 0.7;
    for (unsigned int i = 0; i < positions.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = positions[i].x();
        p.y = positions[i].y();
        p.z = positions[i].z();

        std_msgs::ColorRGBA c;
        c.a = 0.7;
        c.r = colors[i].r;
        c.g = colors[i].g;
        c.b = colors[i].b;

        marker.points.push_back(p);
        marker.colors.push_back(c);
    }
    // markerArr.he
    label_points_pub_.publish(marker);
}

void initLabelPublishers(ros::NodeHandle &nh_priv)
{
    for (auto const &[key, val] : ufo::map::SemanticLabels::LABEL_TO_USED_LABEL_MAP)
    {
        label_publishers_[ufo::map::SemanticLabels::LABEL_TO_STR_MAP.at(key)] = nh_priv.advertise<visualization_msgs::Marker>(ufo::map::SemanticLabels::LABEL_TO_STR_MAP.at(key) + "Label_Point_Markers", 10);
    }
}

void publishLabelPoints(const ros::TimerEvent &event)
{

    std::visit(
        [&](auto &map) {
            using T = std::decay_t<decltype(map)>;
            if constexpr (!std::is_same_v<T, std::monostate>)
            {
                if constexpr (std::is_same_v<T, ufo::map::OccupancyMapSemanticColor>)
                {

                    // Very expensive, todo:: do once for all
                    for (auto &[label_name, label_pub] : label_publishers_)
                    {
                        if (label_pub.getNumSubscribers() == 0)
                        {
                            continue;
                        }
                        std::vector<uint32_t> label_to_find = {ufo::map::SemanticLabels::LABEL_TO_USED_LABEL_MAP.at(ufo::map::SemanticLabels::GetLabelFromString(label_name))};
                        std::vector<ufo::math::Vector3> positions;
                        std::vector<ufo::map::Color> colors;
                        for (auto it = map.beginLabelTree(label_to_find, true, false, false, false, 0), it_end = map.endLabelTree(); it != it_end; ++it)
                        {
                            if (it->labels == nullptr)
                            {
                                // std::cout << "iterator has no labels! " << it.getX() << ' ' << it.getY() << ' ' << it.getZ() << "\n";
                                continue;
                            }
                            positions.push_back(ufo::math::Vector3(it.getX(), it.getY(), it.getZ()));
                            colors.push_back(ufo::map::SemanticLabels::LABEL_TO_COLOR.at(label_to_find[0]));
                        }
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = "map";
                        marker.header.stamp = ros::Time();
                        marker.type = visualization_msgs::Marker::POINTS;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.scale.x = 0.04;
                        marker.scale.y = 0.04;
                        marker.scale.z = 0.04;
                        marker.color.a = 0.7;
                        for (unsigned int i = 0; i < positions.size(); ++i)
                        {
                            geometry_msgs::Point p;
                            p.x = positions[i].x();
                            p.y = positions[i].y();
                            p.z = positions[i].z();

                            std_msgs::ColorRGBA c;
                            c.a = 0.7;
                            c.r = colors[i].r;
                            c.g = colors[i].g;
                            c.b = colors[i].b;

                            marker.points.push_back(p);
                            marker.colors.push_back(c);
                        }
                        // markerArr.he
                        label_pub.publish(marker);
                    }
                }
            }
        },
        ufo_map);
}

void semanticUFOmapEval(sensor_msgs::PointCloud2::ConstPtr const &cloud,
                        geometry_msgs::TransformStamped::ConstPtr const &transform, ufomap_msgs::Labels::ConstPtr const &labelArray)
{
    std::cout << "Callback! \n";
    std::visit(
        [&](auto &map) {
            using T = std::decay_t<decltype(map)>;
            if constexpr (!std::is_same_v<T, std::monostate>)
            {
                //  std::cout << "monostate!\n";
                auto start = std::chrono::system_clock::now();
                ufo::math::Pose6 frame_origin(
                    transform->transform.translation.x, transform->transform.translation.y,
                    transform->transform.translation.z, transform->transform.rotation.w,
                    transform->transform.rotation.x, transform->transform.rotation.y,
                    transform->transform.rotation.z);
                ufo::map::Point3 sensor_origin(transform->transform.translation.x,
                                               transform->transform.translation.y,
                                               transform->transform.translation.z);
                if constexpr (std::is_same_v<T, ufo::map::OccupancyMap>)
                {
                    std::cout << "Occupancymap!\n";
                    ufo::map::PointCloud ufo_cloud;
                    ufomap_ros::rosToUfo(*cloud, ufo_cloud);
                    ufo_cloud.transform(frame_origin, async);

                    map.insertPointCloudDiscrete(sensor_origin, ufo_cloud, max_range, depth,
                                                 simple_ray_casting, early_stopping);
                }
                else if constexpr (std::is_same_v<T, ufo::map::OccupancyMapColor>)
                {
                    std::cout << "Occupancymapcolor!\n";
                    ufo::map::PointCloudColor ufo_cloud;
                    ufomap_ros::rosToUfo(*cloud, ufo_cloud);
                    ufo_cloud.transform(frame_origin, async);

                    map.insertPointCloudDiscrete(sensor_origin, ufo_cloud, max_range, depth,
                                                 simple_ray_casting, early_stopping);
                }
                else if constexpr (std::is_same_v<T, ufo::map::OccupancyMapSemanticColor>)
                {

                    //std::cout << "Occupancymapsemanticcolor!\n";

                    ufo::map::PointCloudSemanticColor ufo_cloud;
                    ufomap_ros::rosToUfo(*cloud, *labelArray, ufo_cloud);

                    //  std::cout << "cloud size = " << ufo_cloud.size() << '\n';
                    ufo_cloud.transform(frame_origin, async);
                    // std::cout << "transform done\n";
                    map.insertPointCloudDiscrete(sensor_origin, ufo_cloud, max_range, depth, simple_ray_casting, early_stopping);
                    // std::cout << "pointcloud inserted\n";
                    //std::cout << "isdense = " << cloud->is_dense << ". Height = " << cloud->height << ". width = " << cloud->width << '\n';

                    if ((num_clouds + 1) % 5 == 0)
                    {
                        ufo::math::Vector3 pos(0.0, 0.0, 0.0);
                        double radius = 10.0;
                        ufo::geometry::Sphere sphere(pos, radius);
                        std::vector<uint32_t> labels_to_find = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
                        //std::cout << "\n \n ITERATING !\n";

                        std::vector<ufo::math::Vector3> positions;
                        std::vector<ufo::map::Color> colors;
                        for (auto it = map.beginLabelTree(labels_to_find, true, false, false, false, 0), it_end = map.endLabelTree(); it != it_end; ++it)
                        {

                            if (it->labels == nullptr)
                            {
                                // std::cout << "iterator has no labels! " << it.getX() << ' ' << it.getY() << ' ' << it.getZ() << "\n";
                                continue;
                            }
                            positions.push_back(ufo::math::Vector3(it.getX(), it.getY(), it.getZ()));
                            // Get highest prob label..
                            float max = 0.0f;
                            unsigned int max_idx = 1;
                            for (unsigned int i = 1; i < ufo::map::SemanticLabels::NUM_LABELS; ++i)
                            {
                                if (it->labels[i] > max)
                                {
                                    max = it->labels[i];
                                    max_idx = i;
                                }
                            }
                            colors.push_back(ufo::map::SemanticLabels::LABEL_TO_COLOR.at(max_idx));
                        }
                        publishLabelMarkerPoints(positions, colors);
                        ufomap_msgs::UFOMapStamped::Ptr msg(new ufomap_msgs::UFOMapStamped);
                        if (ufomap_msgs::ufoToMsg(map, msg->map))
                        {
                            msg->header.stamp = ros::Time::now();
                            msg->header.frame_id = "map";
                            pub_.publish(msg);
                        }
                    }
                }

                std::chrono::duration<double> ufo_elapsed =
                    std::chrono::system_clock::now() - start;

                double elapsed =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                    1000000.0;

                elapsed_ufomap += elapsed;

                max_time = std::max(max_time, elapsed);
                min_time = std::min(min_time, elapsed);

                ++num_clouds;
                std::cout << " Num_clouds = " << num_clouds << '\n';
                /*
                fprintf(stderr,
                        "\33[2K\r[%sTotal %.2f s%s][%sCurrent %.4f ms%s][%sAvg. "
                        "%.4f ms%s][%sMin/Max %.4f/%.4f ms%s][%sCloud %u%s]",
                        CYAN, elapsed_ufomap / 1000.0, RESET, RED, elapsed, RESET, BOLDBLUE,
                        elapsed_ufomap / ((double)num_clouds), RESET, GREEN, min_time, max_time,
                        RESET, MAGENTA, num_clouds, RESET);
*/
                if (publish && 0 == (num_clouds % publish_every_x))
                {
                    ufomap_msgs::UFOMapStamped map_msg;
                    if (ufomap_msgs::ufoToMsg(map, map_msg.map, compress))
                    {
                        map_msg.header.frame_id = "map";
                        map_msg.header.stamp = cloud->header.stamp;
                        pub_.publish(map_msg);
                    }
                }
            }
        },
        ufo_map);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "insertion");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    double label_prob_increase = nh_priv.param("label_prob_increase", 0.6);
    double label_prob_decrease = nh_priv.param("label_prob_decrease", 0.2);
    double resolution = nh_priv.param("resolution", 0.05);
    bool color = nh_priv.param("color", false);
    depth = nh_priv.param("depth", 0);
    max_range = nh_priv.param("max_range", -1.0);
    simple_ray_casting = nh_priv.param("simple_ray_casting", false);
    early_stopping = nh_priv.param("early_stopping", 0);
    async = nh_priv.param("async", false);
    bool automatic_pruning = nh_priv.param("automatic_pruning", true);
    publish = nh_priv.param("publish", true);
    publish_every_x = std::max(1, nh_priv.param("publish_every_x", 1));
    compress = nh_priv.param("compress", true);
    std::string bag_folder =
        nh_priv.param<std::string>("bag_folder", "~/Documents/datasets/");
    std::string bag_file = nh_priv.param<std::string>("bag", "ethz_cow");
    std::string cloud_topic = nh_priv.param<std::string>("cloud_topic", "/camera/depth/points");
    std::string transform_topic =
        nh_priv.param<std::string>("transform_topic", "/tf");
    std::string labels_topic =
        nh_priv.param<std::string>("labels_topic", "/camera/labels");

    fprintf(stderr,
            "Dataset: %s\nUFOMap settings:\n\tColor: %s\n\tAutomatic pruning: "
            "%s\n\tResolution: "
            "%.2f cm\n\tDepth: %u\n\tMax range: %.2f m\n\tRay casting method: "
            "%s\n\tEarly stopping: %u\n\tAsync: %s\n\tPublish: %s\n\tPublish per every %d "
            "cloud\n\tCompress: %s\n",
            bag_file.c_str(), (color ? "On" : "Off"), (automatic_pruning ? "On" : "Off"),
            resolution * 100.0, depth, max_range,
            (simple_ray_casting ? "Simple" : "Normal"), early_stopping,
            (async ? "On" : "Off"), (publish ? "On" : "Off"), publish_every_x,
            (compress ? "On" : "Off"));

    /*std::vector<std::string> topics;
    topics.push_back(cloud_topic);
    topics.push_back(transform_topic);
    */
    //rosbag::View view(bag, rosbag::TopicQuery(topics));

    pub_ = nh_priv.advertise<ufomap_msgs::UFOMapStamped>("map", 10);
    label_marker_pub_ = nh_priv.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    label_points_pub_ = nh_priv.advertise<visualization_msgs::Marker>("Label_Point_Markers", 10);
    initLabelPublishers(nh_priv);

    std::cout << "label_prob_increase = " << label_prob_increase << ". decrease = " << label_prob_decrease << '\n';
    ufo_map.emplace<ufo::map::OccupancyMapSemanticColor>(resolution, 16, automatic_pruning, label_prob_increase, label_prob_decrease);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, cloud_topic, 10);
    message_filters::Subscriber<geometry_msgs::TransformStamped> transformSub(nh, transform_topic, 10);
    message_filters::Subscriber<ufomap_msgs::Labels> labelsSub(nh, labels_topic, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            geometry_msgs::TransformStamped,
                                                            ufomap_msgs::Labels>
        MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointCloudSub, transformSub, labelsSub);

    sync.registerCallback(boost::bind(&semanticUFOmapEval, _1, _2, _3));

    // Publisher timer
    ros::Timer timer = nh_priv.createTimer(ros::Duration(2.0), publishLabelPoints);

    /*
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = "map";
    static_transformStamped.transform.translation.x = 0;
    static_transformStamped.transform.translation.y = 0;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, 90.0);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_tf_broadcaster.sendTransform(static_transformStamped);
*/
    ros::spin();
    /*
    geometry_msgs::TransformStamped::ConstPtr last_transform;
    for (rosbag::MessageInstance const &msg : view)
    {
        if (msg.getTopic() == cloud_topic || ("/" + msg.getTopic() == cloud_topic))
        {
            sensor_msgs::PointCloud2::ConstPtr const cloud =
                msg.instantiate<sensor_msgs::PointCloud2>();
            if (nullptr != cloud)
            {
                cloud_sub.newMessage(cloud);
            }
        }

        if (msg.getTopic() == transform_topic || ("/" + msg.getTopic() == transform_topic))
        {
            geometry_msgs::TransformStamped::ConstPtr const transform =
                msg.instantiate<geometry_msgs::TransformStamped>();
            if (nullptr != transform)
            {
                transform_sub.newMessage(transform);
            }
        }

        if (!ros::ok())
        {
            exit(0);
        }
    }

    bag.close();
*/
    return 0;
}