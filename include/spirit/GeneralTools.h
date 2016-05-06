#ifndef GENERALTOOLS_H__
#define GENERALTOOLS_H__

#include <string>
#include <sys/stat.h>

class GeneralTools {
 public:
  static bool CheckFileExists(const std::string& file_name);
};

#endif  // GENERALTOOLS_H__
