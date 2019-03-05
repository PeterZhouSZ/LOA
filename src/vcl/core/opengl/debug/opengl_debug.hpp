#pragma once

#include "../../../external_lib/headers/glad.hpp"

#include <string>

#define opengl_debug() vcl::check_opengl_error(__FILE__,__PRETTY_FUNCTION__,__LINE__)

namespace vcl
{

void opengl_debug_print_version();
void check_opengl_error(const std::string& file, const std::string& function, int line);

}
