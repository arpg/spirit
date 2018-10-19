#ifndef SP_GENERALTOOLS_H__
#define SP_GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>
#include <chrono>
#include <thread>
#include <pangolin/pangolin.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

typedef std::chrono::high_resolution_clock::time_point spTimestamp;

class spGeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
  static spTimestamp Tick();
  static double Tock_ms(spTimestamp tick_time);
  static double Tock_ns(spTimestamp tick_time);
  static double TickTock_s(spTimestamp tick_time,spTimestamp tock_time);
  static double TickTock_ms(spTimestamp tick_time,spTimestamp tock_time);
  static double TickTock_us(spTimestamp tick_time,spTimestamp tock_time);
  static double Tock_us(spTimestamp tick_time);
  static void Delay_ms(unsigned int delay_time);
  static void Delay_us(unsigned int delay_time);
  static void PlotXY(std::vector<double>& x_axis,std::string xlabel="x", std::string ylabel="y");
};

#endif  // SP_GENSERALTOOLS_H__
