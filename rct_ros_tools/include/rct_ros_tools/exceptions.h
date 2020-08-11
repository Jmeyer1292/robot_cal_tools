#pragma once

#include <stdexcept>
#include <string>

namespace rct_ros_tools
{
/**
 * \brief Thrown during errors in loading or parsing data files
 */
class BadFileException : public std::runtime_error
{
public:
  BadFileException(const std::string& what) : std::runtime_error(what) {}
};

} // namespace rct_ros_tools
