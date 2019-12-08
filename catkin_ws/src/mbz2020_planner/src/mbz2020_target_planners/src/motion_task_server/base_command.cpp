#include <motion_task_server/base_command.hpp>

using namespace MotionTaskServer;

BaseCommand::BaseCommand(ros::NodeHandle& nh) {
    CommandBuffer::initBuffer(nh);
    this->buffer = std::make_unique<CommandBuffer>(CommandBuffer::getBuffer());
}

bool BaseCommand::handle() {
    throw new std::runtime_error("BaseCommand: abstract handle function called");
}
