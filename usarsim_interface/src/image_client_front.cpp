//----------------------------------------------------------------------
//
// ardrone_driver style ROS interface for USARsim UPIS imageserver
// 
// This is based on the imageserver produced by Pras Velagapudi
// The original source (New BSD License) can be obtained from:
// http://code.google.com/p/pkv-ros-pkg/
// Note: The original source did not contain BSD headers
// 
// Modification by Alex Karmazyn:
//	Executables added for targeting specific simulated cameras
//	Host modified to 'USARsim' to allow remapping via /etc/hosts
//  Node naming changed to mimic the ardrone_driver
//
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <vector>

static const int DEFAULT_RATE = 60;
static const int DEFAULT_PORT = 5003;
static const std::string DEFAULT_HOST = "USARsim";
static const std::string DEFAULT_REQUEST = "OK\r\n";

std::vector<unsigned char> intToBytes(int paramInt)
{
     std::vector<unsigned char> arrayOfByte(4);
     for (int i = 0; i < 4; i++)
         arrayOfByte[3 - i] = (paramInt >> (i * 8));
     return arrayOfByte;
}


/**
 * Opens a TCP socket to the desired host.
 *
 * @param host the hostname or IP address of the desired host
 * @param port the port number to connect to at the host
 * @return a file descriptor >=0 if the socket opened, or <0 then the socket failed.
 */
int openSocket(std::string host, int port)
{
    struct sockaddr_in serv_addr;
    struct hostent *server;

    // Create a new socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
        ROS_ERROR("Error opening socket.");
        return -1;
    }

    // Lookup the specified host using DNS
    server = gethostbyname(host.c_str());
    if (server == NULL) 
    {
        ROS_ERROR("No such host.");
        return -1;
    }

    // Set socket timeouts to large, but not infinite wait
    struct timeval timeout;      
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0
        || setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
    {
        ROS_ERROR("Failed to set socket timeouts.");
        return -1;
    }

    // Create an address structure and connect a socket to it
    bzero((char*) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char*)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(port);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    {
        ROS_ERROR("Failed to connect to host: %s[%d].", host.c_str(), port);
        return -1;
    }

    return sockfd;
}

/**
 * Writes an arbitrary string out to the specified socket.
 *
 * @param sockfd the socket that will be written to
 * @param request the string that should be written
 */
void requestImage(int sockfd, std::string request)
{
    int n = -1;
    std::vector<unsigned char> x = intToBytes(0);
    std::string sx(reinterpret_cast<char*>(&x[0]), x.size());
    std::vector<unsigned char> y = intToBytes(0);
    std::string sy(reinterpret_cast<char*>(&y[0]), y.size());
    std::vector<unsigned char> w = intToBytes(320);
    std::string sw(reinterpret_cast<char*>(&w[0]), w.size());
    std::vector<unsigned char> h = intToBytes(240);
    std::string sh(reinterpret_cast<char*>(&h[0]), h.size());
    std::string strStart = "U";
    std::string strTerm  = "\r\n";


    write(sockfd, strStart.c_str(), strStart.size());
    /*n = send(sockfd, x, 4, 0);
    n = send(sockfd, y, 4, 0);
    n = send(sockfd, w, 4, 0);
    n = send(sockfd, z, 4, 0);*/
    write(sockfd, sx.c_str(), sx.size());
    write(sockfd, sy.c_str(), sy.size());
    write(sockfd, sw.c_str(), sw.size());
    write(sockfd, sh.c_str(), sh.size());
    write(sockfd, strTerm.c_str(), strTerm.size());

    /*// Write out the specified request string
    n = 
    if (n < 0)
        ROS_ERROR("Error writing image request to socket.");
    else
        ROS_DEBUG("Requested image.");*/


}

/**
 * Receives and decompresses an ImageServer image.
 *
 * @param sockfd the socket receiving the image
 * @return an OpenCV matrix containing the decompressed image
 */
cv::Mat receiveImage(int sockfd)
{
    int n = -1;
    unsigned char imageType = 0;    
    unsigned int imageSize = 0;

    // Get the type of the image (0: raw, >0: jpeg)
    n = read(sockfd, &imageType, 1);
    if (n < 0) 
    {
        ROS_ERROR("Error reading image type from socket.");
        return cv::Mat();
    }
    else
    ROS_DEBUG("Received image type: %s.", (imageType == 0) ? "RAW" : "JPEG");

    // Get the size of the received image
    n = read(sockfd, &imageSize, 4);
    if (n < 0) 
    {
        ROS_ERROR("Error reading image size from socket.");
        return cv::Mat();
    }
    imageSize = ntohl(imageSize);
    ROS_DEBUG("Received image size: %d.", imageSize);

    // Read the data from the image into a temporary array
    unsigned char image[imageSize];
    unsigned int bytesRead = 0;    
    while(bytesRead < imageSize)
    { 
        n = read(sockfd, image + bytesRead, imageSize - bytesRead);
        bytesRead += n;
        if (n < 0)
        {
            ROS_ERROR("Error reading image from socket.");
            return cv::Mat();
        }
    }
    ROS_DEBUG("Received %d/%d bytes.", bytesRead, imageSize);

    // Convert the image into a cv::Mat, decompressing if necessary
    if (imageType > 0)
        return cv::imdecode(cv::Mat(1, imageSize, CV_8UC1, image), 1);
    else
        return cv::Mat(
            ntohs(((unsigned short*)image)[1]), 
            ntohs(((unsigned short*)image)[0]), 
            CV_8UC3, image + 4);
}

/**
 * Runs a ROS node that receives image data from a USARSim/UPIS ImageServer
 * and converts it into a ROS image stream.
 */
int main(int argc, char** argv)
{
  std::string request_string;
  int request_rate;
  std::string is_host;
  int is_port;

  // Create a node handle and start up ROS
  ros::init(argc, argv, "image_client_front");
  ros::NodeHandle nh;
 
  // Load custom parameters from server
  ros::NodeHandle pnh("~");
  pnh.param<std::string>("host", is_host, DEFAULT_HOST);
  pnh.param<int>("port", is_port, DEFAULT_PORT);
  pnh.param<std::string>("request", request_string, DEFAULT_REQUEST);
  pnh.param<int>("rate", request_rate, DEFAULT_RATE);

  // Create the image transport infrastructure and make a channel
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("ardrone/front/image_raw", 1);
  
  // Open a socket to the USARSIM/UPIS image server
  int sockfd = openSocket(is_host, is_port);
  if (sockfd < 0)
    return -1;

  // Request images at some specified rate
  ros::Rate loop_rate(request_rate);
  while (nh.ok()) {

    // Get the next image from USARSim/UPIS
    requestImage(sockfd, request_string);
    cv::Mat image = receiveImage(sockfd);

    // Convert the image to ROS-compatible format and publish
    IplImage *ipl_img = new IplImage(image);
    sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(ipl_img, "bgr8");
    pub.publish(msg);
    
    // Sleep for a while
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Shut down the socket used for USARSim/UPIS ImageServer
  close(sockfd);
}
