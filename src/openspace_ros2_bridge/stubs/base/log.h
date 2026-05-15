#pragma once
#include <iostream>
#include <sstream>

#define ERT_PLOG_I std::cout
#define ERT_PLOG_D std::cout
#define ERT_PLOG_W std::cout
#define ERT_PLOG_E std::cerr

#define CHECK_NOTNULL(ptr) ((ptr) != nullptr)
