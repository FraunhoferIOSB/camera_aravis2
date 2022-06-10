#pragma once

#include <cstddef>
#include <string>
#include <sstream>
#include <vector>

namespace camera_aravis
{
// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

void parse_string_args(std::string in_arg_string, std::vector<std::string>& out_args)
{
  size_t array_start = 0;
  size_t array_end = in_arg_string.length();
  if (array_start != std::string::npos && array_end != std::string::npos)
  {
    // parse the string into an array of parameters
    std::stringstream ss(in_arg_string.substr(array_start, array_end - array_start));
    while (ss.good())
    {
      std::string temp;
      getline(ss, temp, ',');
      trim(temp);
      out_args.push_back(temp);
    }
  }
  else
  {
    // add just the one argument onto the vector
    out_args.push_back(in_arg_string);
  }
}

}  // end namespace camera_aravis
