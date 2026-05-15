#include <gtest/gtest.h>

#define private public
#include "decision_data/lookup_table.h"


namespace gpal::pnc::planning::Decision {

class LookupTableTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}

  LookupTable1D lookup_table_1d_;
  LookupTable2D lookup_table_2d_;
};

TEST_F(LookupTableTest, LookupTable1D_1) {
  std::vector<double> input_x;
  std::vector<double> input_y;
  lookup_table_1d_.Initialize(input_x, input_y);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable1D_2) {
  lookup_table_1d_.Initialize({1.0, 2.0}, {1.0, 2.0});
  double target_x = 0.0;
  lookup_table_1d_.Lookup(target_x);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable1D_3) {
  lookup_table_1d_.Initialize({1.0, 2.0}, {1.0, 2.0});
  double target_x = 3.0;
  lookup_table_1d_.Lookup(target_x);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable1D_4) {
  lookup_table_1d_.Initialize({1.0, 2.0}, {1.0, 2.0});
  double target_x = 1.5;
  lookup_table_1d_.Lookup(target_x);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_1) {
  std::vector<double> input_x;
  std::vector<double> input_y;
  std::vector<std::vector<double>> data;
  lookup_table_2d_.Initialize(input_x, input_y, data);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_2) {
  std::vector<double> input_x = {1.0, 2.0};
  std::vector<double> input_y = {1.0, 2.0, 3.0};
  std::vector<std::vector<double>> data = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  lookup_table_2d_.Initialize(input_x, input_y, data);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_3) {
  std::vector<double> input_x = {1.0, 2.0};
  std::vector<double> input_y = {1.0, 2.0};
  std::vector<std::vector<double>> data = {{1.0, 2.0}, {4.0, 5.0}};
  lookup_table_2d_.Initialize(input_x, input_y, data);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_4) {
  std::vector<double> input_x = {1.0, 2.0};
  std::vector<double> input_y = {1.0, 2.0};
  std::vector<std::vector<double>> data = {{1.0, 2.0}, {4.0, 5.0}};
  lookup_table_2d_.Initialize(input_x, input_y, data);
  double x = 1.5;
  double y = 1.5;
  lookup_table_2d_.Lookup(x, y);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_5) {
  std::vector<double> input_x = {2.0, 1.0};
  std::vector<double> input_y = {1.0, 2.0};
  std::vector<std::vector<double>> data = {{1.0, 2.0}, {4.0, 5.0}};
  lookup_table_2d_.Initialize(input_x, input_y, data);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_6) {
  std::vector<double> input_x = {1.0, 2.0};
  std::vector<double> input_y = {2.0, 1.0};
  std::vector<std::vector<double>> data = {{1.0, 2.0}, {4.0, 5.0}};
  lookup_table_2d_.Initialize(input_x, input_y, data);
  EXPECT_TRUE(true);
}

TEST_F(LookupTableTest, LookupTable2D_7) {
  std::vector<double> input_x = {1.0};
  std::vector<double> input_y = {2.0};
  std::vector<std::vector<double>> data = {{1.0}};
  lookup_table_2d_.Initialize(input_x, input_y, data);
  double x = 1.0;
  double y = 2.0;
  lookup_table_2d_.Lookup(x, y);
  EXPECT_TRUE(true);
}

}  // namespace gpal::pnc::planning::Decision