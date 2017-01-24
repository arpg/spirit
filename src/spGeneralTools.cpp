#include <spirit/spGeneralTools.h>

// Checks if a file exists in a path
bool spGeneralTools::CheckFileExists(const std::string& file_name) {
  struct stat buffer;
  return(stat(file_name.c_str(),&buffer) == 0);
}

void spGeneralTools::Delay_ms(unsigned int delay_time) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
}

void spGeneralTools::Delay_us(unsigned int delay_time) {
  std::this_thread::sleep_for(std::chrono::microseconds(delay_time));
}

spTimestamp spGeneralTools::Tick(){
  return std::chrono::high_resolution_clock::now();
}

double spGeneralTools::Tock_ms(spTimestamp tick_time) {
  spTimestamp tock_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>( tock_time - tick_time ).count();
}

double spGeneralTools::Tock_us(spTimestamp tick_time) {
  spTimestamp tock_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>( tock_time - tick_time ).count();
}

double spGeneralTools::TickTock_ms(spTimestamp tick_time,spTimestamp tock_time) {
  return std::chrono::duration_cast<std::chrono::milliseconds>( tock_time - tick_time ).count();
}
