/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-13 14:19
#
# Filename: file_manager.hpp
#
# Description:
#
************************************************/

#ifndef VSLAM_TOOLS_FILE_MANAGER_HPP_
#define VSLAM_TOOLS_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>

namespace vslam {
class FileManager {
public:
  static bool CreateFile(std::ofstream &ofs, std::string file_path);
  static bool InitDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path);
};
} // namespace vslam

#endif
