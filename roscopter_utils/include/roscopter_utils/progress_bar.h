#pragma once
#include <vector>
#include <string>
#include <chrono>

class ProgressBar
{
public:
  ProgressBar(){}
  ProgressBar(int total, int barwidth) :
    initialized_(false),
    barwidth_(barwidth),
    total_(total)
  {}
  
  ~ProgressBar()
  {
//    std::cout << std::endl;
  }
  
  void init(int total, int barwidth)
  {
    initialized_ = false;
    barwidth_ = barwidth;
    total_ = total;
    last_completed_ = 0;
  }

  void set_theme_line() { bars_ = {"─", "─", "─", "╾", "╾", "╾", "╾", "━", "═"}; }
  void set_theme_circle() { bars_ = {" ", "◓", "◑", "◒", "◐", "◓", "◑", "◒", "#"}; }
  void set_theme_braille() { bars_ = {" ", "⡀", "⡄", "⡆", "⡇", "⡏", "⡟", "⡿", "⣿" }; }
  void set_theme_braille_spin() { bars_ = {" ", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠇", "⠿" }; }
  
  void print(int completed, double t=NAN)
  {
    if (!initialized_)
    {
      last_print_time_ = std::chrono::system_clock::now();
      start_time_ = std::chrono::system_clock::now();
      initialized_ = true;
      t_start_ = t;
      elapsed_last_ = 0.0;
      t_last_ = t;
      rt_rate_ = 1.0;
      rt_alpha_ = 0.9;
    }
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    // limit printing to about 30 Hz
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count() > 33
        || completed == total_)
    {
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count() / 1000.0;
      last_print_time_ = now;
      std::cout << " \r [";
      double pos = barwidth_ * (completed / (double)total_);
      for (int i = 0; i < barwidth_; ++i)
        if (i < floor(pos)) std::cout << *(bars_.end()-1);
        else if (i == floor(pos)) std::cout << bars_[round((pos - floor(pos)) * (bars_.size() -1))];
        else std::cout << " ";
      std::cout << "]  ";
      printf("%.0f%% ", (completed / (double)total_)*100.0);
      double it_s = completed / elapsed;
      std::string left_stamp = ms_to_stamp(((total_ - completed) / it_s)*1000);
      std::string elapsed_stamp = ms_to_stamp(elapsed * 1000.0);
      if (std::isfinite(t_start_) && std::isfinite(t))
      {
          double rt_factor = (t - t_last_) / (elapsed - elapsed_last_);
          rt_rate_ = rt_alpha_ * rt_rate_ + (1.0 - rt_alpha_) * rt_factor;
          t_last_ = t;
          elapsed_last_ = elapsed;
          printf("[%s<%s, %.2fit/s, %.2fx] ", elapsed_stamp.c_str(), left_stamp.c_str(), it_s, rt_rate_);
      }
          else
      {
          printf("[%s<%s, %.2fit/s] ", elapsed_stamp.c_str(), left_stamp.c_str(), it_s);
      }
      std::cout.flush();
    }
    last_completed_ = completed;
    t_end_ = t;
  }

  void finished()
  {
    print(total_, t_end_);
  }
private:

  std::string ms_to_stamp(int ms)
  {
    if (ms <= 0.0)
    {
      return "";
    }
    int millis = ms % 1000;
    int sec = ((ms - millis) % (60 * 1000)) / 1000;
    int min = ((ms - (millis + sec*1000)) % (60 * 60 * 1000)) / (60*1000);
    int hour = ((ms - (millis + (sec + min*60)*1000)) % (24 * 60 * 60 * 1000)) / (60*60*1000);
    int day = ((ms - (millis + (sec + (min + hour * 60) * 60) * 1000)) / (24 * 60 * 60 * 1000))/(24*60*60*1000);
    char buf[25];
    int n;
    if (day > 0)
      n = sprintf(buf, "%d:%d:%02d:%02d:%03d", day, hour, min, sec, millis);
    else if (hour > 0)
      n = sprintf(buf, "%d:%02d:%02d:%03d", hour, min, sec, millis);
    else if (min > 0)
      n = sprintf(buf, "%d:%02d:%03d", min, sec, millis);
    else if (sec > 0)
      n = sprintf(buf, "%d:%03d", sec, millis);
    else
      n = sprintf(buf, "%d", millis);
    std::string out(buf);
    return out;
  }



  double rt_alpha_;
  double rt_rate_;
  double t_last_;
  double elapsed_last_;
  int barwidth_;
  int total_;
  bool initialized_;
  int last_completed_;
  double t_start_;
  double t_end_;
  std::vector<const char*> bars_ = {" ", "▏", "▎", "▍", "▋", "▋", "▊", "▉", "▉", "█"};
  
  std::chrono::system_clock::time_point start_time_;
  std::chrono::system_clock::time_point last_print_time_;  
};
