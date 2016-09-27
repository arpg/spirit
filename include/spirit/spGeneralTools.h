#ifndef SP_GENERALTOOLS_H__
#define SP_GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>
#include <chrono>
#include <thread>
#include <spirit/Types/spTypes.h>


class spGeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
  static spTimestamp Tick();
  static double Tock_ms(spTimestamp tick_time);
  static double Tock_us(spTimestamp tick_time);
  static void Delay_ms(unsigned int delay_time);
  static void Delay_us(unsigned int delay_time);
};

#endif  // SP_GENSERALTOOLS_H__
