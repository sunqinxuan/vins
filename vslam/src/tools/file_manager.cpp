/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-13 14:18
#
# Filename: file_manager.cpp
#
# Description:
#
************************************************/

#include "tools/file_manager.hpp"
#include <boost/filesystem.hpp>

namespace vslam {
bool FileManager::CreateFile(std::ofstream &ofs, std::string file_path) {
  ofs.close();
  boost::filesystem::remove(file_path.c_str());

  ofs.open(file_path.c_str(), std::ios::out);
  if (!ofs) {
    std::cerr << "无法生成文件: " << std::endl
              << file_path << std::endl
              << std::endl;
    return false;
  }

  return true;
}

bool FileManager::InitDirectory(std::string directory_path,
                                std::string use_for) {
  if (boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::remove_all(directory_path);
  }

  return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path,
                                  std::string use_for) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    std::cerr << "cannot create directory: " << std::endl
              << directory_path << std::endl
              << std::endl;
    return false;
  }

  std::cout << use_for << " storage address：" << std::endl
            << directory_path << std::endl
            << std::endl;
  return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
  if (!boost::filesystem::is_directory(directory_path)) {
    boost::filesystem::create_directory(directory_path);
  }

  if (!boost::filesystem::is_directory(directory_path)) {
    std::cerr << "cannot create directory: " << std::endl
              << directory_path << std::endl
              << std::endl;
    return false;
  }

  return true;
}
} // namespace vslam
