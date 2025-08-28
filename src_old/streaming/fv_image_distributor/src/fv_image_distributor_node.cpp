#include "fv_image_distributor/fv_image_distributor.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>

namespace fv_image_distributor
{

FVImageDistributor::FVImageDistributor()
    : Node("fv_image_distributor"),
      running_(false),
      has_image_(false),
      frame_count_(0),
      average_fps_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing FV Image Distributor C++");
    
    loadParameters();
    initializeSubscriptions();
    startHttpServer();
    
    RCLCPP_INFO(this->get_logger(), "FV Image Distributor C++ initialized successfully");
}

FVImageDistributor::~FVImageDistributor()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down FV Image Distributor C++");
    
    running_ = false;
    stopHttpServer();
    
    if (http_thread_.joinable()) {
        http_thread_.join();
    }
}

void FVImageDistributor::loadParameters()
{
    // Declare parameters with default values
    this->declare_parameter("input_topic", "/fv_mjpeg_image");
    this->declare_parameter("http_port", 8080);
    this->declare_parameter("compressed", false);
    this->declare_parameter("frame_rate", 30.0);
    
    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    http_port_ = this->get_parameter("http_port").as_int();
    compressed_ = this->get_parameter("compressed").as_bool();
    frame_rate_ = this->get_parameter("frame_rate").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Input Topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  HTTP Port: %d", http_port_);
    RCLCPP_INFO(this->get_logger(), "  Compressed: %s", compressed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Frame Rate: %.1f", frame_rate_);
}

void FVImageDistributor::initializeSubscriptions()
{
    if (compressed_) {
        compressed_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic_, 10, std::bind(&FVImageDistributor::compressedImageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Compressed image subscription initialized");
    } else {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_, 10, std::bind(&FVImageDistributor::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Raw image subscription initialized");
    }
}

void FVImageDistributor::startHttpServer()
{
    try {
        acceptor_ = std::make_unique<boost::asio::ip::tcp::acceptor>(io_context_);
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), http_port_);
        acceptor_->open(endpoint.protocol());
        acceptor_->set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
        acceptor_->bind(endpoint);
        acceptor_->listen();
        
        running_ = true;
        http_thread_ = std::thread(&FVImageDistributor::httpWorker, this);
        
        RCLCPP_INFO(this->get_logger(), "HTTP server started on port %d", http_port_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start HTTP server: %s", e.what());
    }
}

void FVImageDistributor::stopHttpServer()
{
    if (acceptor_) {
        acceptor_->close();
    }
    io_context_.stop();
}

void FVImageDistributor::httpWorker()
{
    while (running_) {
        try {
            boost::asio::ip::tcp::socket socket(io_context_);
            acceptor_->accept(socket);
            
            // Handle request in a separate thread
            std::thread([this, socket = std::move(socket)]() mutable {
                handleHttpRequest(std::move(socket));
            }).detach();
        } catch (const std::exception& e) {
            if (running_) {
                RCLCPP_ERROR(this->get_logger(), "HTTP server error: %s", e.what());
            }
        }
    }
}

void FVImageDistributor::handleHttpRequest(boost::asio::ip::tcp::socket socket)
{
    try {
        boost::beast::flat_buffer buffer;
        boost::beast::http::request<boost::beast::http::string_body> req;
        boost::beast::http::read(socket, buffer, req);
        
        std::string path = req.target().to_string();
        
        if (path == "/" || path == "/index.html") {
            // Serve HTML page
            std::string html = createHtmlPage();
            sendHttpResponse(socket, "text/html", html);
        } else if (path == "/image.jpg") {
            // Serve current image
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (has_image_) {
                std::vector<uchar> buffer;
                cv::imencode(".jpg", current_image_, buffer);
                std::string image_data(buffer.begin(), buffer.end());
                sendHttpResponse(socket, "image/jpeg", image_data);
            } else {
                // Send 404 if no image available
                std::string not_found = "No image available";
                sendHttpResponse(socket, "text/plain", not_found);
            }
        } else {
            // 404 Not Found
            std::string not_found = "404 Not Found";
            sendHttpResponse(socket, "text/plain", not_found);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "HTTP request handling error: %s", e.what());
    }
}

void FVImageDistributor::sendHttpResponse(boost::asio::ip::tcp::socket& socket, 
                                         const std::string& content_type, 
                                         const std::string& body)
{
    try {
        boost::beast::http::response<boost::beast::http::string_body> res;
        res.result(boost::beast::http::status::ok);
        res.set(boost::beast::http::field::server, "FV-Image-Distributor");
        res.set(boost::beast::http::field::content_type, content_type);
        res.set(boost::beast::http::field::access_control_allow_origin, "*");
        res.body() = body;
        res.prepare_payload();
        
        boost::beast::http::write(socket, res);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "HTTP response error: %s", e.what());
    }
}

void FVImageDistributor::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        auto cv_image = cv_bridge::toCvShare(msg, "bgr8");
        if (cv_image) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            current_image_ = cv_image->image.clone();
            has_image_ = true;
            last_image_time_ = std::chrono::steady_clock::now();
            
            frame_count_++;
            
            // Log every 30 frames
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Processed frame %lu", frame_count_);
            }
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void FVImageDistributor::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    try {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (!image.empty()) {
            std::lock_guard<std::mutex> lock(image_mutex_);
            current_image_ = image.clone();
            has_image_ = true;
            last_image_time_ = std::chrono::steady_clock::now();
            
            frame_count_++;
            
            // Log every 30 frames
            if (frame_count_ % 30 == 0) {
                RCLCPP_INFO(this->get_logger(), "Processed compressed frame %lu", frame_count_);
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Compressed image processing error: %s", e.what());
    }
}

std::string FVImageDistributor::createHtmlPage()
{
    std::stringstream html;
    html << "<!DOCTYPE html>\n"
         << "<html>\n"
         << "<head>\n"
         << "    <title>FV Image Distributor</title>\n"
         << "    <style>\n"
         << "        body { font-family: Arial, sans-serif; margin: 20px; }\n"
         << "        .container { max-width: 800px; margin: 0 auto; }\n"
         << "        .image-container { text-align: center; margin: 20px 0; }\n"
         << "        img { max-width: 100%; height: auto; border: 1px solid #ccc; }\n"
         << "        .status { background: #f0f0f0; padding: 10px; border-radius: 5px; }\n"
         << "    </style>\n"
         << "</head>\n"
         << "<body>\n"
         << "    <div class=\"container\">\n"
         << "        <h1>FV Image Distributor</h1>\n"
         << "        <div class=\"status\">\n"
         << "            <p><strong>Status:</strong> <span id=\"status\">Connecting...</span></p>\n"
         << "            <p><strong>Frame Count:</strong> <span id=\"frameCount\">0</span></p>\n"
         << "        </div>\n"
         << "        <div class=\"image-container\">\n"
         << "            <img id=\"image\" src=\"/image.jpg\" alt=\"Live Image\" />\n"
         << "        </div>\n"
         << "    </div>\n"
         << "    <script>\n"
         << "        function updateImage() {\n"
         << "            const img = document.getElementById('image');\n"
         << "            const status = document.getElementById('status');\n"
         << "            const frameCount = document.getElementById('frameCount');\n"
         << "            \n"
         << "            img.onload = function() {\n"
         << "                status.textContent = 'Connected';\n"
         << "                frameCount.textContent = parseInt(frameCount.textContent) + 1;\n"
         << "            };\n"
         << "            \n"
         << "            img.onerror = function() {\n"
         << "                status.textContent = 'Error loading image';\n"
         << "            };\n"
         << "            \n"
         << "            img.src = '/image.jpg?' + new Date().getTime();\n"
         << "        }\n"
         << "        \n"
         << "        // Update image every 100ms\n"
         << "        setInterval(updateImage, 100);\n"
         << "        updateImage();\n"
         << "    </script>\n"
         << "</body>\n"
         << "</html>\n";
    
    return html.str();
}

} // namespace fv_image_distributor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<fv_image_distributor::FVImageDistributor>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
