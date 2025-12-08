#pragma once

#include <string>

namespace grillex {

/**
 * Placeholder test function to verify C++ build system is working.
 * Returns a greeting message.
 */
std::string get_greeting();

/**
 * Test function that performs a simple calculation.
 * @param a First number
 * @param b Second number
 * @return Sum of a and b
 */
int add_numbers(int a, int b);

}  // namespace grillex
