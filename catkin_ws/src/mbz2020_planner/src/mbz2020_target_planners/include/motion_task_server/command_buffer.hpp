#ifndef COMMAND_BUFFER_HPP
#define COMMAND_BUFFER_HPP

// system and ros headers
#include <ros/ros.h>

namespace MotionTaskServer {

class CommandBuffer {
    static std::unique_ptr<CommandBuffer> ptr_;
public:
    static CommandBuffer& getBuffer();
    static void initBuffer(ros::NodeHandle& nh); 
private:
    CommandBuffer(ros::NodeHandle& nh);
    ~CommandBuffer();
};
}
#endif // COMMAND_BUFFER_HPP
