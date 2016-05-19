#include <spirit/spGeneralTools.h>

// Checks if a file exists in a path
bool spGeneralTools::CheckFileExists(const std::string& file_name) {
  struct stat buffer;
  return(stat(file_name.c_str(),&buffer) == 0);
}
