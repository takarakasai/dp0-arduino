// ROS2
#include <ros2arduino.h>
#include <ros2/sensor_msgs/Image.hpp>

// Adafruit AMG88XX
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// config for ros2arduino
#define XRCEDDS_PORT  Serial
#define PUBLISH_FREQUENCY 10 //hz

// config for AMG88XX
#define AMG88XX_WIDTH  (8)
#define AMG88XX_HEIGHT (AMG88xx_PIXEL_ARRAY_SIZE / AMG88XX_WIDTH)

// variables for AMG88XX
Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

void publishImage(sensor_msgs::Image* img_msg, void* arg)
{
  (void)(arg);

  img_msg->height = AMG88XX_HEIGHT;
  img_msg->width = AMG88XX_WIDTH;
  strncpy(img_msg->encoding, "32FC1", sizeof(img_msg->encoding));
  img_msg->is_bigendian = 0;
  img_msg->step = 4 * img_msg->width;
  img_msg->data_size = img_msg->height * img_msg->step;
  img_msg->data = reinterpret_cast<uint8_t *>(pixels);

  amg.readPixels(pixels);
}

class Dp0Node : public ros2::Node
{
public:
  Dp0Node()
  : Node("ros2arduino_dp0_node")
  {
    ros2::Publisher<sensor_msgs::Image>* publisher_ = this->createPublisher<sensor_msgs::Image>("image_raw");
    this->createWallFreq(PUBLISH_FREQUENCY, (ros2::CallbackFunc)publishImage, nullptr, publisher_);
  }
};

void setupAmg88xx() {
  bool status = amg.begin();
  while (!status) {
    status = amg.begin();
  }
}

void setup() 
{
  setupAmg88xx();
  
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT); 

  ros2::init(&XRCEDDS_PORT);
}

void loop() 
{
  static Dp0Node dp0_node;

  ros2::spin(&dp0_node);
}
