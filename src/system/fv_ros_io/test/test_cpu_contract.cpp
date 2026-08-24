#include "cpu_contract.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace fv::ros_io::detail
{
namespace
{

TEST(CpuContract, ParsesLinuxCpuLists)
{
  EXPECT_EQ(parseCpuList("0-2,5,7-8\n"), (std::vector<int>{0, 1, 2, 5, 7, 8}));
  EXPECT_EQ(parseCpuList("3,1,2,2"), (std::vector<int>{1, 2, 3}));
  EXPECT_TRUE(parseCpuList("\n").empty());
}

TEST(CpuContract, RejectsInvalidCpuLists)
{
  EXPECT_THROW(parseCpuList("2-1"), std::runtime_error);
  EXPECT_THROW(parseCpuList("1,,2"), std::runtime_error);
  EXPECT_THROW(parseCpuList("cpu3"), std::runtime_error);
}

TEST(CpuContract, ParsesExactlyOneReservedCpu)
{
  EXPECT_EQ(parseReservedCpu("quiet fv_ros_io.cpu=13 isolcpus=domain,13"), 13);
  EXPECT_THROW(parseReservedCpu("quiet isolcpus=domain,13"), std::runtime_error);
  EXPECT_THROW(
    parseReservedCpu("fv_ros_io.cpu=12 fv_ros_io.cpu=13"),
    std::runtime_error);
}

TEST(CpuContract, ValidatesOnlineIsolatedAndAllowedSets)
{
  cpu_set_t allowed;
  CPU_ZERO(&allowed);
  CPU_SET(3, &allowed);

  EXPECT_NO_THROW(validateCpuContract(3, {0, 1, 2, 3}, {3}, allowed));
  EXPECT_THROW(validateCpuContract(3, {0, 1, 2}, {3}, allowed), std::runtime_error);
  EXPECT_THROW(validateCpuContract(3, {0, 1, 2, 3}, {2}, allowed), std::runtime_error);

  CPU_ZERO(&allowed);
  CPU_SET(2, &allowed);
  EXPECT_THROW(validateCpuContract(3, {0, 1, 2, 3}, {3}, allowed), std::runtime_error);
}

}  // namespace
}  // namespace fv::ros_io::detail
