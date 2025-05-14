#ifndef MONKEY_BRAIN_BASIC_LOGGER_SPLIT_HPP
#define MONKEY_BRAIN_BASIC_LOGGER_SPLIT_HPP

#include <cassert>
#include <string_view>
#include <vector>

namespace monkey_brain_basic_logger
{

std::vector<std::string_view> split(std::string_view s, std::string_view delimiter)
{
  assert(delimiter.size() > 0);
  std::vector<std::string_view> tokens;
  std::size_t pos = 0UL;
  while ((pos = s.find(delimiter)) != std::string_view::npos) {
    tokens.push_back(s.substr(0, pos));
    tokens.push_back(delimiter);
    s.remove_prefix(pos + delimiter.length());
  }
  tokens.push_back(s);
  return tokens;
}

} // namespace monkey_brain_basic_logger
#endif // MONKEY_BRAIN_BASIC_LOGGER_SPLIT_HPP
