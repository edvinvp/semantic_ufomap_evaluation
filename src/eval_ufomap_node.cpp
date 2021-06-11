/*
*   Code used for the evaluation of Semantic UFOMap
*   @Edvin von Platen
*/

// UFO
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>
#include <ufo/map/occupancy_map_semantic_color.h>
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
#include <sensor_msgs/Image.h>
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

struct wellford_stat
{
    // Random voxel queries
    // Wellford algorithm for computing online statistics
    // https://en.m.wikipedia.org/wiki/Algorithms_for_calculating_variance
    double count = 0.0;
    double mean = 0.0;
    double M2 = 0.0;

    void update(double data)
    {
        // wellford algorithm update
        count += 1.0;
        double delta = data - mean;
        mean += delta / count;
        double delta2 = data - mean;
        M2 += delta * delta2;
    }

    double calcSTDEV() { return std::sqrt(calcSampleVariance()); }

    double calcVariance() { return M2 / count; }
    double calcSampleVariance() { return M2 / (count - 1.0); }
};

unsigned int num_clouds = 0;
double elapsed_ufomap(0);
unsigned int num_occupied = 0;
unsigned int num_free = 0;
size_t num_nodes = 0;
double max_time = 0.0;
double min_time = std::numeric_limits<double>::max();
bool insert = true;
unsigned int instance_depth;
unsigned int instance_query_depth;
float instance_prob_threshold;

double instance_prob_increase;
double instance_prob_decrease;
double prob_hit;
double prob_miss;

bool automatic_pruning;

ros::Publisher pub_;
ros::Publisher instance_marker_pub_;
ros::Publisher instance_points_pub_;

ros::Timer timer_query;

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
double resolution = 0.02;

double free_thres = 0.5;
double occupied_thres = 0.5;

auto pointcloud_timer = std::chrono::system_clock::now();

std::vector<uint32_t>
    instances_061 = {28974, 37386, 7656, 64749, 6116, 47794, 7666, 61965, 55431, 7066, 35329, 5256, 49388, 2998, 24389, 18946, 4187, 33784, 53930, 9042, 33804, 37256, 29481, 1089, 11337, 40280, 11357, 15314, 62912, 20693, 58129, 26853, 14512, 64217, 31935, 27493, 37244, 61789, 8718, 26658, 5266, 15525, 18931, 43810, 29948, 23623, 39037, 29493, 59981, 19580, 32968, 157, 8330, 55769, 32295, 36292, 39648, 38503, 25864, 27383, 17856, 13943, 34071, 9805, 37998, 40031, 19724, 30050, 59165, 18662, 60223, 2519, 1667, 28079, 25566, 10334, 40793, 3359, 9191, 29664, 17837, 25677, 23122, 55311, 35974, 214, 4847, 40756, 42195, 41949, 5427, 36559, 36874, 55878, 30597, 5851, 25197, 40143, 21779, 18188, 11370, 4723, 34878, 49167, 51313, 21742, 37781, 58518, 54641, 39760, 8280, 45518, 35148, 41153, 38531, 48885, 60456, 4311, 31334, 46137, 47778, 25381, 1312, 18579, 1147, 64193, 12060, 5712, 55409, 40732, 36455, 13669, 18433};
//std::vector<uint32_t> instances_061 = {37386}; //, 7656, 64749, 6116};
std::vector<ufo::map::Color> instance_colors_061 = {{143, 133, 174}, {35, 230, 65}, {45, 165, 221}, {236, 215, 92}, {178, 203, 250}, {201, 250, 235}, {127, 142, 35}, {227, 140, 221}, {27, 129, 176}, {168, 216, 37}, {123, 158, 3}, {113, 80, 146}, {59, 43, 90}, {196, 97, 211}, {211, 190, 56}, {101, 79, 158}, {100, 37, 212}, {166, 190, 189}, {58, 168, 140}, {225, 100, 219}, {197, 0, 179}, {99, 178, 229}, {32, 39, 80}, {174, 29, 58}, {57, 188, 229}, {248, 244, 159}, {250, 240, 208}, {180, 14, 170}, {129, 123, 177}, {88, 28, 56}, {213, 191, 30}, {234, 178, 37}, {195, 110, 30}, {19, 43, 115}, {163, 174, 132}, {159, 202, 87}, {80, 223, 29}, {166, 2, 32}, {106, 59, 108}, {20, 83, 250}, {153, 79, 35}, {96, 112, 110}, {213, 45, 133}, {204, 104, 149}, {149, 22, 68}, {20, 67, 216}, {112, 56, 182}, {109, 30, 203}, {111, 24, 96}, {162, 55, 50}, {170, 142, 113}, {234, 227, 143}, {49, 26, 193}, {223, 49, 100}, {214, 237, 153}, {147, 164, 215}, {158, 222, 250}, {157, 65, 167}, {72, 101, 194}, {103, 93, 43}, {209, 44, 41}, {159, 145, 9}, {66, 134, 51}, {177, 87, 62}, {181, 154, 56}, {151, 10, 206}, {58, 205, 43}, {27, 70, 24}, {34, 185, 16}, {113, 196, 98}, {10, 95, 154}, {112, 47, 14}, {93, 103, 161}, {197, 120, 46}, {250, 129, 217}, {172, 23, 147}, {128, 212, 47}, {158, 92, 122}, {225, 157, 119}, {6, 250, 128}, {171, 88, 229}, {230, 202, 40}, {188, 165, 249}, {159, 124, 133}, {184, 125, 214}, {169, 30, 200}, {32, 96, 49}, {27, 3, 24}, {239, 52, 138}, {86, 24, 221}, {233, 113, 109}, {174, 132, 170}, {30, 16, 21}, {231, 62, 123}, {11, 53, 88}, {242, 161, 92}, {2, 82, 32}, {95, 170, 77}, {195, 165, 244}, {132, 17, 65}, {250, 238, 39}, {170, 170, 55}, {166, 210, 26}, {63, 77, 55}, {19, 71, 178}, {250, 56, 47}, {84, 241, 250}, {89, 74, 243}, {48, 196, 57}, {181, 55, 209}, {131, 46, 76}, {211, 75, 230}, {9, 37, 6}, {50, 67, 51}, {118, 13, 45}, {36, 250, 224}, {152, 14, 94}, {122, 86, 209}, {177, 0, 168}, {153, 155, 99}, {132, 19, 49}, {127, 150, 106}, {87, 113, 150}, {164, 10, 108}, {158, 250, 132}, {147, 45, 16}, {71, 210, 29}, {195, 150, 18}, {94, 123, 120}, {138, 22, 149}, {90, 150, 246}, {5, 194, 240}, {112, 144, 78}};
//std::vector<ufo::map::Color> instance_colors_061 = {{35, 230, 65}}; //, {45, 165, 221}, {236, 215, 92}, {178, 203, 250}};
std::vector<uint32_t> instances_201 = {8080, 33982, 20130, 51943, 37059, 41934, 6546, 42339, 11156, 47619, 29775, 25902, 2940, 51842, 36651, 40455, 32456, 17651, 41593, 32871, 54161, 15813, 64548, 46302, 27431, 36747, 8475, 18179, 2667, 36638, 28430, 34072, 7688, 16184, 49946, 42907, 35112, 3062, 12269, 2104, 19699, 4152, 13397, 45649, 13723, 12471, 14051, 10861, 21420, 62636, 50278, 9488, 46365, 15678, 20424, 6113, 20579, 29521, 41377, 43330, 29229, 58208, 15411, 13004, 64499, 4931, 42902, 36394, 11902, 53131, 10286, 5261, 38217, 59940, 59119, 33359, 42505, 62385, 54616, 3654, 255, 62843, 3561, 63379, 42577, 49279, 25256, 57605, 27757, 11435, 9382, 39255, 34589, 30004, 59485, 3761, 59382, 15637, 64531, 41399, 27108, 28478, 27113, 42942, 40283, 16126, 5439, 60075, 34584, 6674, 48624, 50600, 1132, 6687, 63725, 22823, 46061, 827, 46901, 8131, 6380, 36107, 10045, 6234, 7706, 5716, 53363, 880, 12274, 53403, 24900, 46974, 33143, 64260, 638, 12362, 64085, 4076, 10842, 2572, 60826, 43734, 53985, 51717, 2126, 27678, 37049, 13052, 22868, 41283, 13985, 30503, 12019, 1339, 24188, 49101, 11204, 39206, 64032, 35198, 8656, 30465, 55257, 30690, 4899, 42093, 46220, 35269, 33578, 54435, 2231, 50075, 11251, 53448, 28616, 13188, 38939, 95, 35318, 2774, 82, 17685, 45900, 40492, 65122, 14578, 50532, 5073, 60670, 14605, 1433, 56169, 26396, 24160, 31645, 65123, 6059, 16425, 16570, 34300, 27462, 32482, 3603, 39342, 61168, 57474, 11868, 36837, 5816, 54410, 11935, 14674, 53417, 89, 12160, 26976, 47212, 64939, 15157, 38368, 43952, 55268, 43448, 854, 51029, 4952, 5024, 35116, 6749, 61914, 49800, 16969, 65443, 47864, 14022, 53416, 63663, 7329, 28641, 13652, 63268, 25177, 32754, 402, 59296, 1204, 55770};
std::vector<ufo::map::Color> instance_colors_201 = {{250, 112, 68}, {115, 99, 45}, {67, 162, 200}, {177, 159, 242}, {126, 46, 246}, {142, 190, 172}, {246, 197, 109}, {144, 62, 101}, {37, 178, 143}, {162, 151, 163}, {101, 45, 180}, {87, 175, 161}, {234, 183, 236}, {177, 79, 225}, {124, 190, 158}, {137, 116, 108}, {110, 76, 18}, {59, 22, 112}, {141, 113, 16}, {111, 164, 198}, {185, 152, 208}, {53, 39, 218}, {220, 243, 192}, {158, 8, 0}, {92, 217, 84}, {125, 29, 236}, {250, 181, 191}, {60, 247, 17}, {8, 47, 201}, {124, 171, 248}, {96, 96, 56}, {115, 198, 85}, {25, 110, 37}, {54, 96, 12}, {170, 190, 55}, {146, 87, 24}, {119, 90, 60}, {235, 46, 31}, {41, 115, 178}, {231, 192, 28}, {66, 32, 7}, {13, 86, 63}, {45, 33, 109}, {155, 203, 55}, {112, 174, 239}, {42, 26, 126}, {47, 99, 17}, {36, 152, 56}, {72, 33, 109}, {214, 166, 13}, {171, 232, 17}, {31, 202, 244}, {158, 61, 145}, {52, 176, 68}, {68, 177, 91}, {20, 3, 169}, {69, 45, 103}, {100, 55, 194}, {140, 167, 108}, {147, 200, 91}, {99, 36, 154}, {199, 159, 13}, {51, 192, 82}, {43, 206, 43}, {220, 198, 77}, {15, 245, 151}, {146, 81, 216}, {123, 225, 34}, {40, 56, 201}, {181, 241, 113}, {34, 121, 80}, {17, 36, 135}, {130, 42, 19}, {205, 98, 200}, {104, 179, 63}, {113, 87, 174}, {144, 220, 141}, {213, 212, 133}, {187, 33, 102}, {237, 50, 44}, {225, 90, 114}, {215, 88, 7}, {236, 227, 59}, {217, 37, 234}, {145, 35, 52}, {168, 110, 183}, {85, 113, 199}, {197, 122, 145}, {93, 239, 121}, {38, 171, 131}, {31, 104, 193}, {133, 137, 211}, {117, 145, 201}, {101, 247, 31}, {203, 214, 240}, {11, 224, 29}, {203, 128, 47}, {52, 145, 88}, {220, 227, 221}, {140, 189, 150}, {91, 174, 167}, {96, 135, 151}, {91, 183, 145}, {146, 115, 5}, {136, 231, 117}, {54, 41, 190}, {243, 23, 250}, {205, 224, 228}, {117, 141, 150}, {21, 224, 212}, {102, 191, 161}, {173, 7, 232}, {3, 1, 37}, {247, 74, 32}, {87, 42, 108}, {77, 24, 41}, {157, 49, 169}, {2, 7, 181}, {160, 47, 57}, {27, 4, 108}, {246, 57, 72}, {122, 234, 49}, {33, 167, 82}, {245, 190, 106}, {25, 126, 167}, {18, 174, 54}, {182, 182, 138}, {2, 46, 73}, {41, 121, 100}, {182, 212, 181}, {84, 70, 127}, {160, 111, 173}, {112, 146, 28}, {220, 5, 182}, {1, 100, 59}, {41, 203, 166}, {219, 108, 223}, {13, 4, 49}, {36, 133, 108}, {7, 217, 78}, {208, 137, 6}, {149, 34, 53}, {184, 227, 125}, {176, 211, 151}, {231, 208, 165}, {93, 169, 193}, {126, 29, 56}, {140, 250, 210}, {77, 58, 241}, {140, 76, 7}, {51, 146, 162}, {103, 162, 197}, {40, 163, 156}, {229, 50, 214}, {81, 210, 22}, {82, 182, 14}, {37, 222, 121}, {133, 96, 239}, {219, 72, 130}, {119, 175, 27}, {28, 221, 40}, {103, 131, 124}, {189, 79, 67}, {104, 53, 26}, {15, 214, 209}, {143, 72, 157}, {157, 177, 93}, {119, 244, 37}, {114, 36, 74}, {186, 129, 30}, {6, 169, 98}, {171, 49, 39}, {27, 131, 158}, {182, 247, 2}, {96, 250, 77}, {44, 98, 67}, {132, 140, 159}, {147, 142, 210}, {120, 28, 86}, {8, 135, 20}, {148, 100, 172}, {59, 46, 60}, {156, 168, 138}, {137, 138, 101}, {67, 109, 162}, {49, 39, 234}, {172, 208, 141}, {241, 216, 245}, {208, 7, 157}, {49, 63, 2}, {229, 126, 234}, {192, 108, 49}, {89, 99, 122}, {81, 188, 158}, {107, 121, 103}, {248, 232, 43}, {19, 215, 203}, {186, 157, 160}, {55, 176, 195}, {116, 161, 173}, {92, 244, 191}, {110, 101, 234}, {11, 102, 129}, {133, 206, 135}, {209, 169, 115}, {68, 171, 41}, {40, 26, 226}, {125, 101, 68}, {19, 5, 96}, {186, 114, 125}, {40, 86, 109}, {49, 110, 187}, {93, 185, 231}, {21, 91, 211}, {161, 69, 42}, {91, 66, 119}, {161, 56, 235}, {222, 80, 75}, {50, 237, 222}, {130, 179, 107}, {149, 229, 249}, {189, 91, 129}, {148, 48, 134}, {227, 132, 100}, {174, 133, 209}, {16, 13, 39}, {241, 173, 208}, {119, 92, 183}, {22, 34, 209}, {212, 70, 157}, {170, 65, 30}, {56, 249, 145}, {224, 37, 94}, {163, 117, 153}, {47, 72, 20}, {182, 221, 224}, {217, 250, 111}, {24, 38, 135}, {97, 17, 167}, {45, 244, 71}, {216, 189, 200}, {85, 53, 23}, {111, 63, 80}, {225, 214, 34}, {139, 98, 115}, {3, 55, 245}, {88, 109, 91}};

std::vector<uint32_t> instances_201_single = {8080};
std::vector<ufo::map::Color> instance_colors_201_single = {{250, 112, 68}};

std::vector<uint32_t> instances_201_top = {
    8080,
    33982,
    20130,
    41593,
    64548, 36638};
std::vector<ufo::map::Color> instance_colors_201_top = {
    {250, 112, 68},
    {115, 99, 45},
    {67, 162, 200},
    {141, 113, 16},
    {220, 243, 192},
    {124, 171, 248}};
/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(boost::shared_ptr<M const> const &msg)
    {
        this->signalMessage(msg);
    }
};

void countInstanceNodes(const ros::TimerEvent &event)
{
    std::visit(
        [&](auto &map)
        {
            using T = std::decay_t<decltype(map)>;
            if constexpr (!std::is_same_v<T, std::monostate>)
            {
                if constexpr (std::is_same_v<T, ufo::map::OccupancyMapSemanticColor>)
                {

                    auto start = std::chrono::system_clock::now();
                    std::map<unsigned int, unsigned int> instance_node_count;

                    for (ufo::map::InstanceType instance : instances_201)
                    {
                        unsigned int emergency = 0;
                        std::vector<ufo::map::InstanceType> i_vec = {instance};
                        instance_node_count.emplace(instance, 0);
                        for (auto it = map.beginInstanceTree(i_vec, instance_depth, instance_query_depth, instance_prob_threshold,
                                                             true, false, false, false, 0),
                                  it_end = map.endInstanceTree();
                             it != it_end; ++it)
                        {
                            instance_node_count[instance] += 1;
                        }
                    }
                    std::cout << "Counts: \n";
                    for (auto &[i, c] : instance_node_count)
                    {
                        std::cout << i << ": " << c << "\n";
                    }
                    std::chrono::duration<double> ufo_elapsed =
                        std::chrono::system_clock::now() - start;

                    double elapsed =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                        1000000.0;
                }
            }
        },
        ufo_map);
}

void publishInstanceMarkerPos(std::vector<ufo::math::Vector3> const &positions, std::vector<ufo::map::Color> const &colors)
{
    visualization_msgs::MarkerArray markerArr;
    markerArr.markers.resize(positions.size());
    std::cout << "positions size = " << positions.size() << ". Colors = " << colors.size() << '\n';

    for (unsigned int i = 0; i < positions.size(); ++i)
    {
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
    instance_marker_pub_.publish(markerArr);
}

void publishInstanceMarkerPoints(std::vector<ufo::math::Vector3> const &positions, std::vector<ufo::map::Color> const &colors)
{
    std::cout << "publishing positions and colors of size " << positions.size() << '\n';
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    double scale = resolution * std::pow(2.0, instance_query_depth);
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.a = 1.0;
    for (unsigned int i = 0; i < positions.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = positions[i].x();
        p.y = positions[i].y();
        p.z = positions[i].z();

        std_msgs::ColorRGBA c;
        c.a = 1.0;
        c.r = colors[i].r;
        c.g = colors[i].g;
        c.b = colors[i].b;

        marker.points.push_back(p);
        marker.colors.push_back(c);
    }
    // markerArr.he
    instance_points_pub_.publish(marker);
}

static double toLogit(double prob) { return std::log(prob / (1.0 - prob)); }

void publishInstancePoints(const ros::TimerEvent &event)
{

    std::visit(
        [&](auto &map)
        {
            using T = std::decay_t<decltype(map)>;
            if constexpr (!std::is_same_v<T, std::monostate>)
            {
                if constexpr (std::is_same_v<T, ufo::map::OccupancyMapSemanticColor>)
                {

                    auto start = std::chrono::system_clock::now();
                    if (instance_points_pub_.getNumSubscribers() == 0)
                    {
                        return;
                    }

                    std::vector<ufo::math::Vector3> positions;
                    std::vector<ufo::map::Color> colors;
                    for (auto it = map.beginInstanceTree(instances_201, instance_depth, instance_query_depth, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                    {

                        // Get the color, first valid
                        unsigned int count_instance = 0;
                        unsigned int max_idx = 0;
                        float max_prob = 0.0f;
                        for (uint32_t const &instance : instances_201)
                        {
                            auto contains = it->instances.find(instance);
                            if (contains != it->instances.end())
                            {
                                if (contains->second > toLogit(instance_prob_threshold))
                                {
                                    if (max_prob < contains->second)
                                    {
                                        max_prob = contains->second;
                                        max_idx = count_instance;
                                    }
                                }
                            }
                            ++count_instance;
                        }

                        colors.push_back(instance_colors_201[max_idx]);
                        positions.push_back(ufo::math::Vector3(it.getX(), it.getY(), it.getZ()));
                    }
                    publishInstanceMarkerPoints(positions, colors);

                    std::chrono::duration<double> ufo_elapsed =
                        std::chrono::system_clock::now() - start;

                    double elapsed =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                        1000000.0;
                }
            }
        },
        ufo_map);
}

std::vector<int> instances_n = {1, 5, 20, 70};
std::map<int, std::vector<double>> search_instances_data;
std::map<int, std::vector<uint32_t>> search_instances;

void printQueries()
{

    for (const int n : instances_n)
    {
        std::cout << "n = " << n;
        for (int i = 0; i < search_instances_data[n].size(); ++i)
        {
            std::cout << search_instances_data[n][i] << ',' << ' ';
        }
        std::cout << '\n';
    }
}

void printMeasureQuerySpeed(std::map<uint32_t, std::vector<double>> &query_timings,
                            std::map<uint32_t, std::vector<double>> &query_timings_first_node_ret,
                            std::map<uint32_t, uint32_t> &instance_counts, int depth)
{
    std::cout << "num_clouds = " << num_clouds << '\n';
    for (auto instance : instances_201_top)
    {
        std::cout << "instance_" << instance << "_n = " << instance_counts[instance] << '\n';
        std::cout << "instance_" << instance << "_d" << depth << " = [";
        for (int i = 0; i < query_timings[instance].size(); ++i)
        {
            if (i == query_timings[instance].size() - 1)
            {
                std::cout << query_timings[instance][i] << ']';
            }
            else
            {
                std::cout << query_timings[instance][i] << ',' << ' ';
            }
        }
        std::cout << '\n';
    }

    std::cout << "instance_all"
              << "_n = " << instance_counts[0] << '\n';
    std::cout << "instance_all"
              << "_d" << depth << " = [";
    for (int i = 0; i < query_timings[0].size(); ++i)
    {
        if (i == query_timings[0].size() - 1)
        {
            std::cout << query_timings[0][i] << ']';
        }
        else
        {
            std::cout << query_timings[0][i] << ',' << ' ';
        }
    }
    std::cout << '\n';
    for (auto instance : instances_201_top)
    {
        std::cout << "instance_first_" << instance << "_d" << depth << " = [";
        for (int i = 0; i < query_timings_first_node_ret[instance].size(); ++i)
        {
            if (query_timings_first_node_ret[instance].size() - 1 == i)
            {
                std::cout << query_timings_first_node_ret[instance][i] << ']';
            }
            else
            {
                std::cout << query_timings_first_node_ret[instance][i] << ',' << ' ';
            }
        }
        std::cout << '\n';
    }
}

struct wellford_stat integration_timing;

int times_queried = 0;
void measureQuerySpeed(const ros::TimerEvent &event)
{
    std::chrono::duration<double> pointcloud_elapsed =
        std::chrono::system_clock::now() - pointcloud_timer;
    // time in milliseconds
    double elapsed_check =
        std::chrono::duration_cast<std::chrono::nanoseconds>(pointcloud_elapsed).count() /
        1000000.0;
    std::cout << "Measure Query Speed elapsed = " << elapsed_check << '\n';
    timer_query.start();
    // four seconds since the last pointcloud
    if (elapsed_check > 4000)
    {
        std::cout << "inside \n";
        std::visit(
            [&](auto &map)
            {
                using T = std::decay_t<decltype(map)>;
                if constexpr (!std::is_same_v<T, std::monostate>)
                {
                    if constexpr (std::is_same_v<T, ufo::map::OccupancyMapSemanticColor>)
                    {
                        const bool do_benchmark = true;

                        if constexpr (do_benchmark)
                        {
                            int max_depth = 5;
                            for (int query_depth = 0; query_depth < max_depth; ++query_depth)
                            {
                                std::map<uint32_t, std::vector<double>> query_timings;
                                std::map<uint32_t, std::vector<double>> query_timings_first_node_ret;
                                std::map<uint32_t, uint32_t> instance_counts;

                                int samples = 10;
                                for (int n = 0; n < samples; ++n)
                                {
                                    for (int i = 0; i < instances_201_top.size(); ++i)
                                    {
                                        ufo::map::InstanceType inst = instances_201_top[i];
                                        std::vector<uint32_t> i_vec = {inst};
                                        uint32_t count = 0;
                                        auto start = std::chrono::system_clock::now();
                                        for (auto it = map.beginInstanceTree(i_vec, instance_depth, query_depth, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                                        {
                                            count++;
                                        }
                                        std::chrono::duration<double> ufo_elapsed =
                                            std::chrono::system_clock::now() - start;
                                        instance_counts.emplace(inst, count);
                                        double elapsed =
                                            std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                                            1000000.0;
                                        query_timings[instances_201_top[i]].push_back(elapsed);
                                    }

                                    // Time all 6 instances toghether
                                    uint32_t count_all = 0;
                                    auto start = std::chrono::system_clock::now();
                                    for (auto it = map.beginInstanceTree(instances_201_top, instance_depth, query_depth, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                                    {
                                        count_all++;
                                    }
                                    std::chrono::duration<double> ufo_elapsed =
                                        std::chrono::system_clock::now() - start;
                                    instance_counts.emplace(0, count_all);
                                    double elapsed =
                                        std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                                        1000000.0;
                                    query_timings[0].push_back(elapsed);

                                    // First node found timings
                                    for (int i = 0; i < instances_201_top.size(); ++i)
                                    {
                                        std::vector<uint32_t> i_vec = {instances_201_top[i]};
                                        auto start = std::chrono::system_clock::now();
                                        for (auto it = map.beginInstanceTree(i_vec, instance_depth, query_depth, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                                        {
                                            break;
                                        }
                                        std::chrono::duration<double> ufo_elapsed =
                                            std::chrono::system_clock::now() - start;

                                        double elapsed =
                                            std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                                            1000000.0;
                                        query_timings_first_node_ret[instances_201_top[i]].push_back(elapsed);
                                    }
                                }
                                printMeasureQuerySpeed(query_timings, query_timings_first_node_ret, instance_counts, query_depth);
                            }

                            // Mean instances per laef
                            double instanceCount = 0;
                            double nodeCount = 0;
                            for (auto it = map.beginInstanceTree(instances_201, instance_depth, 0, 0.1, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                            {
                                nodeCount += 1.0;
                                instanceCount += (double)it->instances.size();
                            }
                            std::cout << "MEAN INSTANCES PER LEAF = " << instanceCount / nodeCount << '\n';

                            // These are specific to SceneNN 201
                            std::default_random_engine generator;
                            // These are far greater than any map value
                            double x_max = -1000000.0;
                            double x_min = 1000000.0;
                            double y_max = -1000000.0;
                            double y_min = 1000000.0;
                            double z_max = -1000000.0;
                            double z_min = 1000000.0;

                            int seed = 0;
                            std::mt19937 gen_x(seed);
                            std::mt19937 gen_y(seed + 1);
                            std::mt19937 gen_z(seed + 2);

                            // 201 specific BBox
                            x_max = 5.0;
                            x_min = -1.3;
                            y_max = 2.4;
                            y_min = -0.35;
                            z_max = 5.3;
                            z_min = -2.1;

                            std::uniform_real_distribution<double> x_distribution(x_min, x_max);
                            std::uniform_real_distribution<double> y_distribution(y_min, y_max);
                            std::uniform_real_distribution<double> z_distribution(z_min, z_max);
                            struct wellford_stat stat_uniform_all;
                            struct wellford_stat stat_uniform_d0;
                            struct wellford_stat stat_uniform_d1;
                            struct wellford_stat stat_uniform_d2;
                            struct wellford_stat stat_uniform_d3;
                            struct wellford_stat stat_uniform_d4;
                            struct wellford_stat stat_uniform_d5;
                            struct wellford_stat stat_uniform_d6;
                            struct wellford_stat stat_uniform_d7;
                            struct wellford_stat stat_uniform_d8;
                            struct wellford_stat stat_uniform_d9;
                            struct wellford_stat stat_uniform_d10;
                            struct wellford_stat stat_adjacent;
                            int n = 10000000;
                            // warmup
                            for (int i = 0; i < 10; ++i)
                            {
                                ufo::map::Point3 p(x_distribution(gen_x), y_distribution(gen_y), z_distribution(gen_z));
                                map.hasInstance(p, 0, instances_201[0], instance_prob_threshold);
                            }

                            for (int i = 0; i < n; ++i)
                            {

                                ufo::map::Point3 p(x_distribution(gen_x), y_distribution(gen_y), z_distribution(gen_z));
                                auto start = std::chrono::system_clock::now();
                                auto res = map.hasInstance(p, 0, instances_201[0], instance_prob_threshold);
                                std::chrono::duration<double> ufo_elapsed =
                                    std::chrono::system_clock::now() - start;
                                double elapsed =
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                                    1000000.0;
                                // wellford algorithm update
                                if (res.second == 0)
                                {
                                    stat_uniform_d0.update(elapsed);
                                }
                                else if (res.second == 1)
                                {
                                    stat_uniform_d1.update(elapsed);
                                }
                                else if (res.second == 2)
                                {
                                    stat_uniform_d2.update(elapsed);
                                }
                                else if (res.second == 3)
                                {
                                    stat_uniform_d3.update(elapsed);
                                }
                                else if (res.second == 4)
                                {
                                    stat_uniform_d4.update(elapsed);
                                }
                                else if (res.second == 5)
                                {
                                    stat_uniform_d5.update(elapsed);
                                }
                                else if (res.second == 6)
                                {
                                    stat_uniform_d6.update(elapsed);
                                }
                                else if (res.second == 7)
                                {
                                    stat_uniform_d7.update(elapsed);
                                }
                                else if (res.second == 8)
                                {
                                    stat_uniform_d8.update(elapsed);
                                }
                                else if (res.second == 9)
                                {
                                    stat_uniform_d9.update(elapsed);
                                }
                                else if (res.second == 10)
                                {
                                    stat_uniform_d10.update(elapsed);
                                }
                                stat_uniform_all.update(elapsed);

                                start = std::chrono::system_clock::now();
                                ufo::map::Point3 p_adj(p.x() + 0.02, p.y(), p.z());
                                map.hasInstance(p_adj, 0, instances_201[0], instance_prob_threshold);

                                ufo_elapsed =
                                    std::chrono::system_clock::now() - start;
                                elapsed =
                                    std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                                    1000000.0;
                                // wellford algorithm update
                                stat_adjacent.update(elapsed);
                            }
                            std::cout << "UNIFORM_QUERIES_all_mean = " << stat_uniform_all.mean << "\n sample Variance_all = " << stat_uniform_all.calcSampleVariance() << "\n stdev_all = " << stat_uniform_all.calcSTDEV() << "\n count_all = " << stat_uniform_all.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D0_mean = " << stat_uniform_d0.mean << "\n sample Variance_d0 = " << stat_uniform_d0.calcSampleVariance() << "\n stdev_d0 = " << stat_uniform_d0.calcSTDEV() << "\n count_d0 = " << stat_uniform_d0.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D1_mean = " << stat_uniform_d1.mean << "\n sample Variance_d1 = " << stat_uniform_d1.calcSampleVariance() << "\n stdev_d1 = " << stat_uniform_d1.calcSTDEV() << "\n count_d1 = " << stat_uniform_d1.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D2_mean = " << stat_uniform_d2.mean << "\n sample Variance_d2 = " << stat_uniform_d2.calcSampleVariance() << "\n stdev_d2 = " << stat_uniform_d2.calcSTDEV() << "\n count_d2 = " << stat_uniform_d2.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D3_mean = " << stat_uniform_d3.mean << "\n sample Variance_d3 = " << stat_uniform_d3.calcSampleVariance() << "\n stdev_d3 = " << stat_uniform_d3.calcSTDEV() << "\n count_d3 = " << stat_uniform_d3.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D4_mean = " << stat_uniform_d4.mean << "\n sample Variance_d4 = " << stat_uniform_d4.calcSampleVariance() << "\n stdev_d4 = " << stat_uniform_d4.calcSTDEV() << "\n count_d4 = " << stat_uniform_d4.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D5_mean = " << stat_uniform_d5.mean << "\n sample Variance_d5 = " << stat_uniform_d5.calcSampleVariance() << "\n stdev_d5 = " << stat_uniform_d5.calcSTDEV() << "\n count_d5 = " << stat_uniform_d5.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D6_mean = " << stat_uniform_d6.mean << "\n sample Variance_d6 = " << stat_uniform_d6.calcSampleVariance() << "\n stdev_d6 = " << stat_uniform_d6.calcSTDEV() << "\n count_d6 = " << stat_uniform_d6.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D7_mean = " << stat_uniform_d7.mean << "\n sample Variance_d7 = " << stat_uniform_d7.calcSampleVariance() << "\n stdev_d7 = " << stat_uniform_d7.calcSTDEV() << "\n count_d7 = " << stat_uniform_d7.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D8_mean = " << stat_uniform_d8.mean << "\n sample Variance_d8 = " << stat_uniform_d8.calcSampleVariance() << "\n stdev_d8 = " << stat_uniform_d8.calcSTDEV() << "\n count_d8 = " << stat_uniform_d8.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D9_mean = " << stat_uniform_d9.mean << "\n sample Variance_d9 = " << stat_uniform_d9.calcSampleVariance() << "\n stdev_d9 = " << stat_uniform_d9.calcSTDEV() << "\n count_d9 = " << stat_uniform_d9.count << '\n';
                            std::cout << "UNIFORM_QUERIES_D10_mean = " << stat_uniform_d10.mean << "\n sample Variance_d10 = " << stat_uniform_d10.calcSampleVariance() << "\n stdev_d10 = " << stat_uniform_d10.calcSTDEV() << "\n count_d10 = " << stat_uniform_d10.count << '\n';
                            std::cout << "ADJACENT_QUERIES_mean = " << stat_adjacent.mean << "\n adj_sample Variance = " << stat_adjacent.calcSampleVariance() << "\n adj_stdev = " << stat_adjacent.calcSTDEV() << "adj_count = " << stat_adjacent.count << '\n';

                            // Memory calc, both inner nodes and leafs
                            auto [mem_usage, inner_node_count, leaf_node_count] = map.getMemoryUsage();
                            std::cout << "MEM_USAGE = " << mem_usage << "\n inner_nodes_count = " << inner_node_count << "\n leaf_node_count = " << leaf_node_count << '\n';
                        }
                        else
                        {
                            /* Relies on map members
                            // Memory calc, both inner nodes and leafs
                            auto [mem_usage, inner_node_count, leaf_node_count] = map.getMemoryUsage();
                            std::cout << " RES = " << resolution << ".  INSTANCE_DEPTH = " << instance_depth << "\n . MEM_USAGE = " << mem_usage << "\n inner_nodes_count = " << inner_node_count << "\n leaf_node_count = " << leaf_node_count << '\n';
                            std::cout << "Integration Timing: " << integration_timing.mean << ".  SD = " << integration_timing.calcSTDEV() << ". Count = " << integration_timing.count << '\n';
                            std::cout << "Integration Timing: MAX = " << max_time << ". MIN = " << min_time << "\n";
                            std::cout << "Instance Fusion: " << map.instance_fusion_stat.mean << ". SD = " << map.instance_fusion_stat.calcSTDEV() << ". Count = " << map.instance_fusion_stat.count << '\n';
                            std::cout << "Instance Fusion: MAX = " << map.max_fusion << ". MIN = " << map.min_fusion << "\n";
                            std::cout << "Instance Insertion: " << map.instance_insertion_stat.mean << ". SD = " << map.instance_insertion_stat.calcSTDEV() << ". Count = " << map.instance_insertion_stat.count << '\n';
                            std::cout << "Instance Insertion: MAX = " << map.max_ins_insert << ". MIN = " << map.min_ins_insert << '\n';
                            std::cout << "Parent Propagation: " << map.parent_propagation_stat.mean << ". SD = " << map.parent_propagation_stat.calcSTDEV() << ". Count = " << map.parent_propagation_stat.count << '\n'
                                      << '\n';
                            std::cout << "Parent Propagation: MAX = " << map.max_par_prop << ". MIN = " << map.min_par_prop << "\n";

                            std::vector<ufo::map::InstanceType> first;
                            std::vector<ufo::map::InstanceType> five;
                            std::vector<ufo::map::InstanceType> twenty;
                            std::vector<ufo::map::InstanceType> seventy;

                            for (int i = 0; i < 70; ++i)
                            {
                                if (i < 1)
                                {
                                    first.push_back(instances_061[i]);
                                }
                                if (i < 5)
                                {
                                    five.push_back(instances_061[i]);
                                }
                                if (i < 20)
                                {
                                    twenty.push_back(instances_061[i]);
                                }
                                if (i < 70)
                                {
                                    seventy.push_back(instances_061[i]);
                                }
                            }
                            int count_first = 0;
                            int count_five = 0;
                            int count_twenty = 0;
                            int count_seventy = 0;
                            for (auto it = map.beginInstanceTree(first, instance_depth, 0, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                            {
                                count_first++;
                            }
                            for (auto it = map.beginInstanceTree(five, instance_depth, 0, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                            {
                                count_five++;
                            }
                            for (auto it = map.beginInstanceTree(twenty, instance_depth, 0, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                            {
                                count_twenty++;
                            }
                            for (auto it = map.beginInstanceTree(seventy, instance_depth, 0, instance_prob_threshold, true, false, false, false, 0), it_end = map.endInstanceTree(); it != it_end; ++it)
                            {
                                count_seventy++;
                            }
                            std::cout << "First   = " << count_first << '\n';
                            std::cout << "Five    = " << count_five << '\n';
                            std::cout << "Twenty  = " << count_twenty << '\n';
                            std::cout << "Seventy = " << count_seventy << '\n';
                            */
                        }
                    }
                }
            },
            ufo_map);
    }
}

void semanticUFOmapScenennEval(sensor_msgs::PointCloud2::ConstPtr const &cloud, geometry_msgs::TransformStamped::ConstPtr const &transform, sensor_msgs::Image::ConstPtr const &instance_image)
{

    std::visit(
        [&](auto &map)
        {
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
                    ufo::map::PointCloud ufo_cloud;
                    ufomap_ros::rosToUfo(*cloud, ufo_cloud);
                    ufo_cloud.transform(frame_origin, async);

                    map.insertPointCloudDiscrete(sensor_origin, ufo_cloud, max_range, depth,
                                                 simple_ray_casting, early_stopping);
                }
                else if constexpr (std::is_same_v<T, ufo::map::OccupancyMapColor>)
                {
                    ufo::map::PointCloudColor ufo_cloud;
                    ufomap_ros::rosToUfo(*cloud, ufo_cloud);
                    ufo_cloud.transform(frame_origin, async);

                    map.insertPointCloudDiscrete(sensor_origin, ufo_cloud, max_range, depth,
                                                 simple_ray_casting, early_stopping);
                }
                else if constexpr (std::is_same_v<T, ufo::map::OccupancyMapSemanticColor>)
                {
                    ufo::map::PointCloudSemanticColor ufo_cloud;
                    ufomap_ros::rosToUfo(*cloud, *instance_image, ufo_cloud);
                    ufo_cloud.transform(frame_origin, async);
                    map.insertPointCloudDiscrete(sensor_origin, ufo_cloud, max_range, depth, simple_ray_casting, early_stopping);
                }

                std::chrono::duration<double> ufo_elapsed =
                    std::chrono::system_clock::now() - start;

                double elapsed =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(ufo_elapsed).count() /
                    1000000.0;
                integration_timing.update(elapsed);
                pointcloud_timer = std::chrono::system_clock::now();

                elapsed_ufomap += elapsed;

                max_time = std::max(max_time, elapsed);
                min_time = std::min(min_time, elapsed);

                ++num_clouds;
                std::cout << num_clouds << ". Timing = " << elapsed_ufomap / ((double)num_clouds) << '\n';

                //fprintf(stderr,
                //        "\33[2K\r[%sTotal %.2f s%s][%sCurrent %.4f ms%s][%sAvg. "
                //        "%.4f ms%s][%sMin/Max %.4f/%.4f ms%s][%sCloud %u%s]",
                //        CYAN, elapsed_ufomap / 1000.0, RESET, RED, elapsed, RESET, BOLDBLUE,
                //        elapsed_ufomap / ((double)num_clouds), RESET, GREEN, min_time, max_time,
                //        RESET, MAGENTA, num_clouds, RESET);

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

    instance_prob_increase = nh_priv.param("instance_prob_increase", 0.7);
    instance_prob_decrease = nh_priv.param("instance_prob_decrease", 0.4);
    prob_hit = nh_priv.param("prob_hit", 0.7);
    prob_miss = nh_priv.param("prob_miss", 0.4);
    resolution = nh_priv.param("resolution", 0.05);
    bool color = nh_priv.param("color", false);
    depth = nh_priv.param("depth", 0);
    max_range = nh_priv.param("max_range", -1.0);
    simple_ray_casting = nh_priv.param("simple_ray_casting", false);
    early_stopping = nh_priv.param("early_stopping", 0);
    async = nh_priv.param("async", false);
    automatic_pruning = nh_priv.param("automatic_pruning", true);
    publish = nh_priv.param("publish", true);
    publish_every_x = std::max(1, nh_priv.param("publish_every_x", 1));
    compress = nh_priv.param("compress", true);
    std::string bag_folder =
        nh_priv.param<std::string>("bag_folder", "~/Documents/datasets/");
    std::string bag_file = nh_priv.param<std::string>("bag", "ethz_cow");
    std::string cloud_topic = nh_priv.param<std::string>("cloud_topic", "/camera/depth/points");
    std::string transform_topic =
        nh_priv.param<std::string>("transform_topic", "/tf");
    std::string instances_topic =
        nh_priv.param<std::string>("instances_topic", "/camera/instances");
    std::string instance_image_topic =
        nh_priv.param<std::string>("instance_image_topic", "/camera/instances/image_raw");

    instance_depth = nh_priv.param("instance_depth", 1);
    instance_query_depth = nh_priv.param("instance_query_depth", 1);
    instance_prob_threshold = nh_priv.param("instance_prob_threshold", 0.7);

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

    pub_ = nh_priv.advertise<ufomap_msgs::UFOMapStamped>("map", 10);
    instance_points_pub_ = nh_priv.advertise<visualization_msgs::Marker>("Instance_Point_Markers", 10);

    std::cout << "instance_prob_increase = " << instance_prob_increase << ". decrease = " << instance_prob_decrease << '\n';

    ufo_map.emplace<ufo::map::OccupancyMapSemanticColor>(resolution, 16, instance_depth, automatic_pruning, instance_prob_increase, instance_prob_decrease, occupied_thres, free_thres, prob_hit, prob_miss);

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub(nh, cloud_topic, 10);
    message_filters::Subscriber<geometry_msgs::TransformStamped> transformSub(nh, transform_topic, 10);
    message_filters::Subscriber<ufomap_msgs::Labels> instancesSub(nh, instances_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> instanceImageSub(nh, instance_image_topic, 10);

    for (const int n : instances_n)
    {
        search_instances_data.emplace(n, std::vector<double>());
        search_instances.emplace(n, std::vector<uint32_t>());

        for (unsigned int i = 0; i < n; ++i)
        {
            search_instances[n].push_back(instances_201[i]);
        }
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            geometry_msgs::TransformStamped,
                                                            sensor_msgs::Image>
        MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pointCloudSub, transformSub, instanceImageSub);

    sync.registerCallback(boost::bind(&semanticUFOmapScenennEval, _1, _2, _3));

    // Publisher timer
    ros::Timer timer = nh_priv.createTimer(ros::Duration(2.0), publishInstancePoints);
    //ros::Timer timer2 = nh_priv.createTimer(ros::Duration(1.0), countInstanceNodes);
    timer_query = nh_priv.createTimer(ros::Duration(2.0), measureQuerySpeed);
    ros::spin();

    return 0;
}