#ifndef BASE_COMMAND_HPP
#define BASE_COMMAND_HPP

// system and ros headers
#include <ros/ros.h>

// command topic buffer
#include <motion_task_server/command_buffer.hpp>

namespace MotionTaskServer {

class BaseCommand {
public:
    BaseCommand(ros::NodeHandle& nh);
    virtual bool handle();
private:
    std::unique_ptr<CommandBuffer> buffer; 
};
}
#endif // BASE_COMMAND_HPP
