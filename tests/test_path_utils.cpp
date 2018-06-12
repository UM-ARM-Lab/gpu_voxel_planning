#include "path_utils.hpp"
#include <gtest/gtest.h>
#include "arc_utilities/eigen_helpers.hpp"

bool approxEq(double a, double b)
{
    return std::abs(a-b) < 0.000001;
}





TEST(PathUtils, dist2d)
{
    std::vector<double> a{0.0, 0.0};
    std::vector<double> b{3.0, 4.0};
    std::vector<double> c{0.3, 0.4};
    std::vector<double> d{-3.0, -4.0};

    EXPECT_TRUE(approxEq(PathUtils::dist(a,b), 5.0));
    
    EXPECT_TRUE(approxEq(PathUtils::dist(a,c), 0.5));
    EXPECT_TRUE(approxEq(PathUtils::dist(a,d), 5.0));
    EXPECT_TRUE(approxEq(PathUtils::dist(b,d), 10.0));
}


TEST(PathUtils, dist7d)
{
    std::vector<double> a{0.6541, 0.6892, 0.7482, 0.4505, 0.0838, 0.2290, 0.9133};
    std::vector<double> b{0.1524, 0.8258, 0.5383, 0.9961, 0.0782, 0.4427, 0.1067};

    EXPECT_TRUE(approxEq(PathUtils::dist(a,b), 1.143854199625109));

}

TEST(PathUtils, densify)
{
    std::vector<double> a{0.0, 0.0};
    std::vector<double> b{1.0, 0.0};
    Path path{a, b};
    EXPECT_EQ(5, PathUtils::densify(path, 0.3).size());
}

TEST(PathUtils, followPartial_2_points)
{
    std::vector<double> a{0.0, 0.0};
    std::vector<double> b{1.0, 0.0};
    Path path{a, b};

    Path ppath = PathUtils::followPartial(path, 0.5);
    EXPECT_EQ(2, ppath.size());
    EXPECT_EQ(0.0, ppath[0][0]);
    EXPECT_EQ(0.0, ppath[0][1]);
    EXPECT_EQ(0.5, ppath[1][0]);
    EXPECT_EQ(0.0, ppath[1][1]);
}


TEST(PathUtils, followPartial_3_points)
{
    std::vector<double> a{0.0, 0.0};
    std::vector<double> b{1.0, 0.0};
    std::vector<double> c{1.0, 1.0};
    Path path{a, b, c};

    Path shortpath = PathUtils::followPartial(path, 0.5);
    EXPECT_EQ(2, shortpath.size());


    Path ppath = PathUtils::followPartial(path, 1.5);
    EXPECT_EQ(3, ppath.size());
    EXPECT_EQ(0.0, ppath[0][0]);
    EXPECT_EQ(0.0, ppath[0][1]);
    EXPECT_EQ(1.0, ppath[1][0]);
    EXPECT_EQ(0.0, ppath[1][1]);
    EXPECT_EQ(1.0, ppath[2][0]);
    EXPECT_EQ(0.5, ppath[2][1]);
}


TEST(PathUtils, followPartial_3_points_all_the_way)
{
    std::vector<double> a{0.0, 0.0};
    std::vector<double> b{1.0, 0.0};
    std::vector<double> c{1.0, 1.0};
    Path path{a, b, c};

    Path ppath = PathUtils::followPartial(path, 10.5);
    EXPECT_EQ(3, ppath.size());
    EXPECT_EQ(0.0, ppath[0][0]);
    EXPECT_EQ(0.0, ppath[0][1]);
    EXPECT_EQ(1.0, ppath[1][0]);
    EXPECT_EQ(0.0, ppath[1][1]);
    EXPECT_EQ(1.0, ppath[2][0]);
    EXPECT_EQ(1.0, ppath[2][1]);
}

GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
