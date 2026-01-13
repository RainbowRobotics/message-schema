#ifndef TIMER_QUEUE_H
#define TIMER_QUEUE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <map>
#include <set>
#include <atomic>
#include <chrono>
#include <cstdint>

class TimerQueue
{
public:
  using TaskId = uint64_t;
  static constexpr TaskId INVALID_TASK_ID = 0;

  TimerQueue() : thread_([this] { run(); }) {}

  ~TimerQueue() { stop(); }

  TimerQueue(const TimerQueue&) = delete;
  TimerQueue& operator=(const TimerQueue&) = delete;

  // Schedule a delayed task, returns task_id for cancellation
  template<typename Func>
  TaskId schedule(std::chrono::milliseconds delay, Func&& func)
  {
    auto deadline = std::chrono::steady_clock::now() + delay;
    TaskId id;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (stopping_) return INVALID_TASK_ID;

      id = ++next_id_;
      tasks_.emplace(deadline, TaskInfo{id, std::forward<Func>(func)});
      active_ids_.insert(id);
    }
    cv_.notify_one();
    return id;
  }

  // Cancel a scheduled task
  bool cancel(TaskId id)
  {
    if (id == INVALID_TASK_ID) return false;

    std::lock_guard<std::mutex> lock(mtx_);
    auto it = active_ids_.find(id);
    if(it != active_ids_.end())
    {
      active_ids_.erase(it);
      cv_.notify_one();
      return true;
    }
    return false;
  }

  // Stop all tasks and shutdown
  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if(stopping_)
      {
        return;
      }
      stopping_ = true;
      active_ids_.clear();
    }
    cv_.notify_one();
    if (thread_.joinable())
      thread_.join();
  }

  bool is_running() const { return !stopping_; }

private:
  struct TaskInfo
  {
    TaskId id;
    std::function<void()> func;
  };

  void run()
  {
    std::unique_lock<std::mutex> lock(mtx_);
    while(!stopping_)
    {
      cleanup_cancelled();

      if(tasks_.empty())
      {
        cv_.wait(lock);
        continue;
      }

      auto it = tasks_.begin();
      auto deadline = it->first;

      if(cv_.wait_until(lock, deadline, [this] { return stopping_ || needs_wakeup(); }))
      {
        if(stopping_)
        {
          break;
        }
        continue;
      }

      auto task = std::move(it->second);
      tasks_.erase(it);

      if(active_ids_.count(task.id))
      {
        active_ids_.erase(task.id);
        lock.unlock();
        task.func();
        lock.lock();
      }
    }
  }

  void cleanup_cancelled()
  {
    for(auto it = tasks_.begin(); it != tasks_.end(); )
    {
      if(active_ids_.count(it->second.id) == 0)
      {
        it = tasks_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  bool needs_wakeup() const
  {
    return tasks_.empty() || active_ids_.count(tasks_.begin()->second.id) == 0;
  }

  using TimePoint = std::chrono::steady_clock::time_point;
  std::multimap<TimePoint, TaskInfo> tasks_;
  std::set<TaskId> active_ids_;

  std::mutex mtx_;
  std::condition_variable cv_;
  std::thread thread_;

  TaskId next_id_ = 0;
  bool stopping_ = false;
};

#endif // TIMER_QUEUE_H
