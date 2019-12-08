#ifndef TASK_STATES_HPP
#define TASK_STATES_HPP

namespace MotionTaskServer { 
    enum class TaskStates { 
        running,
        done,
        aborted, 
        failed 
    };
}

#endif // TASK_STATES_HPP
