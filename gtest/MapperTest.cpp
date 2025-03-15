
#include <gtest/gtest.h>
#include "Mapper.h"

TEST(MapperTest, VectorNormalization) {
    Eigen::Vector3f v(3.0f, 4.0f, 0.0f);
    Eigen::Vector3f normalized = v.normalized();
    EXPECT_FLOAT_EQ(normalized.norm(), 1.0f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
