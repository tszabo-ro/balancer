#ifndef task_h
#define task_h

#include <Arduino.h>

class Task
{
public:
  Task(float execution_rate_hz, void (*executor)(), void (*reset)() = [](){})
  : time_between_calls_(static_cast<unsigned int>(1000.0/execution_rate_hz)) // [ms]
  , last_call_time_(0)
  , executor_(executor)
  , reset_(reset)
  {
  }

  void run(unsigned long now)
  {
    if (now < last_call_time_ + time_between_calls_)
    {
      return;
    }
    last_call_time_ = now;

    executor_();
  }

  void reset()
  {
    reset_();
  }

private:
  const unsigned int time_between_calls_;
  unsigned long last_call_time_;

  void (*executor_)();
  void (*reset_)();
};

#endif