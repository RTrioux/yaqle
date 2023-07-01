#include <cmath>
#include <gtest/gtest.h>

// Include the header files of the components you want to test
#include "quat.hpp"

using namespace yaqle;

// Entry point for the test executable
int main(int argc, char **argv)
{
    // Initialize Google Test
    ::testing::InitGoogleTest(&argc, argv);

    // Run the tests
    return RUN_ALL_TESTS();
}
