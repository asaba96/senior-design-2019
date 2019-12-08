#include <motion_task_server/command_buffer.hpp>

using namespace MotionTaskServer;

CommandBuffer& CommandBuffer::getBuffer() {
    if (ptr_ == nullptr) {
        throw new std::runtime_error("CommandBuffer: call initBuffer before retrieving buffer");
    }
    return *ptr_;
}

void CommandBuffer::initBuffer(ros::NodeHandle& nh) {
    if (ptr_ == nullptr) {
        ptr_ = std::make_unique<CommandBuffer>(nh);
    }
}

CommandBuffer::CommandBuffer(ros::NodeHandle& nh) {
    // will do stuff here later
}
