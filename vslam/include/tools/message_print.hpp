/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2023-06-15 11:57
#
# Filename: message_print.hpp
#
# Description:
#
************************************************/

#ifndef MESSAGE_PRINT_HPP
#define MESSAGE_PRINT_HPP

#include <iostream>
#include <string>

namespace vslam {
template <typename A> inline void REMIND(A a) {
  std::cerr << std::fixed << "\033[1m\33[32m" << a << "\033[0m" << std::endl;
}
template <typename A, typename B> inline void REMIND(A a, B b) {
  std::cerr << std::fixed << "\033[1m\33[32m" << a << " " << b << "\033[0m"
            << std::endl;
}
template <typename A, typename B, typename C>
inline void REMIND(A a, B b, C c) {
  std::cerr << std::fixed << " \033[1m\33[32m" << a << " " << b << " " << c
            << "\033[0m" << std::endl;
}

template <typename A> inline void INFO(A a) {
  std::cerr << std::fixed << " \033[1m\33[37m" << a << "\033[0m" << std::endl;
}
template <typename A, typename B> inline void INFO(A a, B b) {
  std::cerr << std::fixed << " \033[1m\33[37m" << a << " " << b << "\033[0m"
            << std::endl;
}

template <typename A, typename B, typename C> inline void INFO(A a, B b, C c) {
  std::cerr << std::fixed << " \033[1m\33[37m" << a << " " << b << " " << c
            << "\033[0m" << std::endl;
}

template <typename A> inline void WARNING(A a) {
  std::cerr << std::fixed << "\033[1m\33[33m" << a << "\033[0m" << std::endl;
}
template <typename A, typename B> inline void WARNING(A a, B b) {
  std::cerr << "\033[1m\33[33m" << a << " " << b << "\033[0m" << std::endl;
}
template <typename A, typename B, typename C>
inline void WARNING(A a, B b, C c) {
  std::cerr << std::fixed << " \033[1m\33[33m" << a << " " << b << " " << c
            << "\033[0m" << std::endl;
}

template <typename A> inline void ERROR(A a) {
  std::cerr << std::fixed << "\033[1m\33[31m" << a << "\033[0m" << std::endl;
}
template <typename A, typename B> inline void ERROR(A a, B b) {
  std::cerr << "\033[1m\33[31m" << a << " " << b << "\033[0m" << std::endl;
}
template <typename A, typename B, typename C> inline void ERROR(A a, B b, C c) {
  std::cerr << "\033[1m\33[31m" << a << " " << b << " " << c << "\033[0m"
            << std::endl;
}
template <typename A> inline void DEBUG(A a) {
  std::cerr << std::fixed << "\033[1m\033[47m\33[34m" << a << "\033[0m"
            << std::endl;
}
template <typename A, typename B> inline void DEBUG(A a, B b) {
  std::cerr << std::fixed << "\033[1m\033[47m\33[34m" << a << " " << b
            << "\033[0m" << std::endl;
}
template <typename A, typename B, typename C> inline void DEBUG(A a, B b, C c) {
  std::cerr << "\033[1m\033[47m\33[34m" << a << " " << b << " " << c
            << "\033[0m" << std::endl;
}
} // namespace vslam
#endif
