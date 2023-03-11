#include "quad_utils/terrain_map_publisher.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace ros2_quadruped {
namespace quad_utils {

TerrainMapPublisher::TerrainMapPublisher() : rclcpp::Node("terrain_map_publisher"),
    terrain_map_(grid_map::GridMap({"z", "nx", "ny", "nz", "z_filt", "nx_filt", "ny_filt", "nz_filt"}))
{
    map_converter_ = std::make_shared<MeshToGridMapConverter>(this);

    // Load rosparams from parameter server
    std::string terrain_map_topic;
    std::string image_topic;

    declare_parameter("terrain_map_raw",  rclcpp::ParameterValue("terrain_map_raw"));
    declare_parameter("map_frame", rclcpp::ParameterValue("map"));
    declare_parameter("update_rate", rclcpp::ParameterValue(10.0));
    declare_parameter("resolution", rclcpp::ParameterValue(0.2));
    declare_parameter("map_data_source", rclcpp::ParameterValue("internal"));
    declare_parameter("terrain_type", rclcpp::ParameterValue("slope"));

    get_parameter("terrain_map_raw", terrain_map_topic);
    get_parameter("map_frame", map_frame_);
    get_parameter("update_rate", update_rate_);
    get_parameter("resolution", resolution_);
    get_parameter("update_rate", update_rate_);
    get_parameter("terrain_type", terrain_type_);

    // Setup pubs and subs
    terrain_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(terrain_map_topic, 1);

    // Add image subscriber if data source requests an image
    if (map_data_source_.compare("image") == 0) {
        declare_parameter("image", "image");
        declare_parameter("min_height", 0.0);
        declare_parameter("max_height", 1.0);

        get_parameter("image", image_topic);
        get_parameter("min_height", min_height_);
        get_parameter("max_height", max_height_);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic, 1, std::bind(&TerrainMapPublisher::loadMapFromImage, this, std::placeholders::_1));
    }

    // Initialize the elevation layer on the terrain map
    terrain_map_.setBasicLayers({"z", "nx", "ny", "nz", "z_filt", "nx_filt", "ny_filt", "nz_filt"});
    timer_ = create_wall_timer(1000ms, std::bind(&TerrainMapPublisher::HandleTimerCallback, this));
}

TerrainMapPublisher::~TerrainMapPublisher() 
{
}


void TerrainMapPublisher::updateParams() 
{
    declare_parameter("obstacle_x", rclcpp::ParameterValue(2.0));
    declare_parameter("obstacle_y", rclcpp::ParameterValue(2.0));
    declare_parameter("obstacle_height", rclcpp::ParameterValue(0.5));
    declare_parameter("obstacle_radius", rclcpp::ParameterValue(1.0));
    declare_parameter("step1_x", rclcpp::ParameterValue(4.0));
    declare_parameter("step1_height", rclcpp::ParameterValue(0.3));
    declare_parameter("step2_x", rclcpp::ParameterValue(6.0));
    declare_parameter("step2_height", rclcpp::ParameterValue(-0.3));

    get_parameter("obstacle_x", obstacle_.x);
    get_parameter("obstacle_y", obstacle_.y);
    get_parameter("obstacle_height", obstacle_.height);
    get_parameter("obstacle_radius", obstacle_.radius);
    get_parameter("step1_x", step1_.x);
    get_parameter("step1_height", step1_.height);
    get_parameter("step2_x", step2_.height);
    get_parameter("step2_height", step2_.height);
}

void TerrainMapPublisher::createMap() 
{
    // Set initial map parameters and geometry
    terrain_map_.setFrameId(map_frame_);
    terrain_map_.setGeometry(
        grid_map::Length(24.0, 12.0), resolution_,
        grid_map::Position(-0.5 * resolution_, -0.5 * resolution_));
    RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
            terrain_map_.getLength().x(), terrain_map_.getLength().y(),
            terrain_map_.getSize()(0), terrain_map_.getSize()(1));
}

void TerrainMapPublisher::updateMap() 
{
    // Add terrain info
    for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        terrain_map_.getPosition(*it, position);
        double x_diff = position.x() - obstacle_.x;
        double y_diff = position.y() - obstacle_.y;

        if (x_diff * x_diff + y_diff * y_diff <=
            obstacle_.radius * obstacle_.radius) {
            terrain_map_.at("z", *it) = obstacle_.height;
            terrain_map_.at("z_filt", *it) = obstacle_.height;
        } else {
            terrain_map_.at("z", *it) = 0.0;
            terrain_map_.at("z_filt", *it) = 0.0;
        }

        if (position.x() >= step1_.x) {
            terrain_map_.at("z", *it) += step1_.height;
            terrain_map_.at("z_filt", *it) += step1_.height;
        }

        if (position.x() >= step2_.x) {
            terrain_map_.at("z", *it) += step2_.height;
            terrain_map_.at("z_filt", *it) += step2_.height;
        }

        terrain_map_.at("nx", *it) = 0.0;
        terrain_map_.at("ny", *it) = 0.0;
        terrain_map_.at("nz", *it) = 1.0;

        terrain_map_.at("nx_filt", *it) = 0.0;
        terrain_map_.at("ny_filt", *it) = 0.0;
        terrain_map_.at("nz_filt", *it) = 1.0;
    }
}

std::vector<std::vector<double>> TerrainMapPublisher::loadCSV(std::string filename) 
{
    std::vector<std::vector<double>> data;
    std::ifstream inputFile(filename);
    int l = 0;

    while (inputFile) {
        l++;
        std::string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
        std::istringstream ss(s);
        std::vector<double> record;

        while (ss) {
            std::string line;
            if (!getline(ss, line, ',')) break;
                try {
                    record.push_back(stod(line));
                } catch (const std::invalid_argument e) {
                    std::cout << "NaN found in file " << filename << " line " << l
                            << std::endl;
                    e.what();
                }
            }
            data.push_back(record);
        }
    }

    if (!inputFile.eof()) {
        std::cerr << "Could not read file " << filename << "\n";
        std::__throw_invalid_argument("File not found.");
    }

    return data;
}

void TerrainMapPublisher::loadMapFromCSV() 
{
    // Load in all terrain data
    std::string package_path = ament_index_cpp::get_package_share_directory("quad_utils");

    std::vector<std::vector<double>> x_data = loadCSV(package_path + "/data/" + terrain_type_ + "/x_data.csv");
    std::vector<std::vector<double>> y_data = loadCSV(package_path + "/data/" + terrain_type_ + "/y_data.csv");
    std::vector<std::vector<double>> z_data = loadCSV(package_path + "/data/" + terrain_type_ + "/z_data.csv");
    std::vector<std::vector<double>> nx_data = loadCSV(package_path + "/data/" + terrain_type_ + "/nx_data.csv");
    std::vector<std::vector<double>> ny_data = loadCSV(package_path + "/data/" + terrain_type_ + "/ny_data.csv");
    std::vector<std::vector<double>> nz_data = loadCSV(package_path + "/data/" + terrain_type_ + "/nz_data.csv");
    std::vector<std::vector<double>> z_data_filt = loadCSV(package_path + "/data/" + terrain_type_ + "/z_data_filt.csv");
    std::vector<std::vector<double>> nx_data_filt = loadCSV(package_path + "/data/" + terrain_type_ + "/nx_data_filt.csv");
    std::vector<std::vector<double>> ny_data_filt = loadCSV(package_path + "/data/" + terrain_type_ + "/ny_data_filt.csv");
    std::vector<std::vector<double>> nz_data_filt = loadCSV(package_path + "/data/" + terrain_type_ + "/nz_data_filt.csv");

    // Grab map length and resolution parameters, make sure resolution is square
    // (and align grid centers with data points)
    int x_size = z_data[0].size();
    int y_size = z_data.size();
    float x_res = x_data[0][1] - x_data[0][0];
    float y_res = y_data[1][0] - y_data[0][0];
    double x_length = x_data[0].back() - x_data[0].front() + x_res;
    double y_length = y_data.back()[0] - y_data.front()[0] + y_res;
    if (x_res != y_res) {
        throw std::runtime_error(
            "Map did not have square elements, make sure x and y resolution are "
            "equal.");
    }

    // Initialize the map
    terrain_map_.setFrameId(map_frame_);
    terrain_map_.setGeometry(
        grid_map::Length(x_length, y_length), x_res,
        grid_map::Position(x_data[0].front() - 0.5 * x_res + 0.5 * x_length,
                            y_data.front()[0] - 0.5 * y_res + 0.5 * y_length));
    RCLCPP_INFO(this->get_logger(),"Created map with size %f x %f m (%i x %i cells).",
            terrain_map_.getLength().x(), terrain_map_.getLength().y(),
            terrain_map_.getSize()(0), terrain_map_.getSize()(1));

    // Load in the elevation and slope data
    for (grid_map::GridMapIterator iterator(terrain_map_); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        grid_map::Position position;
        terrain_map_.getPosition(*iterator, position);
        terrain_map_.at("z", *iterator) = z_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
        terrain_map_.at("nx", *iterator) = nx_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
        terrain_map_.at("ny", *iterator) = ny_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
        terrain_map_.at("nz", *iterator) = nz_data[(y_size - 1) - index[1]][(x_size - 1) - index[0]];

        terrain_map_.at("z_filt", *iterator) = z_data_filt[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
        terrain_map_.at("nx_filt", *iterator) = nx_data_filt[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
        terrain_map_.at("ny_filt", *iterator) = ny_data_filt[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
        terrain_map_.at("nz_filt", *iterator) = nz_data_filt[(y_size - 1) - index[1]][(x_size - 1) - index[0]];
    }
}

void TerrainMapPublisher::loadMapFromImage(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    // Initialize the map from the image message if not already done so
    if (!map_initialized_) {
        grid_map::GridMapRosConverter::initializeFromImage(*msg, resolution_, terrain_map_);
        RCLCPP_INFO(this->get_logger(),"Initialized map with size %f x %f m (%i x %i cells).",
                terrain_map_.getLength().x(), terrain_map_.getLength().y(),
                terrain_map_.getSize()(0), terrain_map_.getSize()(1));
        map_initialized_ = true;
    }

    // Add the data layers
    grid_map::GridMapRosConverter::addLayerFromImage(*msg, "z", terrain_map_, min_height_, max_height_);
    grid_map::GridMapRosConverter::addColorLayerFromImage(*msg, "color", terrain_map_);

    // Add in slope information
    for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        terrain_map_.at("nx", *it) = 0.0;
        terrain_map_.at("ny", *it) = 0.0;
        terrain_map_.at("nz", *it) = 1.0;
    }

    // Move the map to place starting location at (0,0)
    grid_map::Position offset = {4.5, 0.0};
    terrain_map_.setPosition(offset);
}

void TerrainMapPublisher::publishMap() 
{
    // Set the time at which the map was published
    auto terrain_map_msg =  grid_map::GridMapRosConverter::toMessage(terrain_map_);
    terrain_map_msg->header.stamp = this->get_clock()->now();
    terrain_map_pub_->publish(std::move(terrain_map_msg));
}

void TerrainMapPublisher::HandleTimerCallback() 
{
    if (!load_map_finished_) {
        // Either wait for an image to show up on the topic or create a map from
        // scratch
        if (map_data_source_.compare("image") == 0) {
            is_map_type_image_ = true;
        } else if (map_data_source_.compare("csv") == 0) {
            loadMapFromCSV();
        } else {
            createMap();
        }
    }

    updateParams();

    if (map_data_source_.compare("internal") == 0) {
        updateMap();
    }
    publishMap();
}

}  // namespace quad_utils
}  // namespace ros2_quadruped