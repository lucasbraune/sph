#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>

TEST_CASE("Addition")
{
  REQUIRE(1+1 == 2);
}