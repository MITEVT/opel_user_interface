#include "util.h"
#include "unity.h"
#include "unity_fixture.h"

TEST_GROUP(Util_Test);

TEST_SETUP(Util_Test) {
  	
}

TEST_TEAR_DOWN(Util_Test) {

}

TEST(Util_Test, test_itoa) {

	char str[20];

	// Test Decimal Conversion
	itoa(20, str, 10);
	TEST_ASSERT_EQUAL_STRING("20", str);

	itoa(20, str, 16);
	TEST_ASSERT_EQUAL_STRING("14", str);

	itoa(6, str, 2);
	TEST_ASSERT_EQUAL_STRING("110", str);
	
}

TEST(Util_Test, test_reverse) {
	char str[5] = "Hello";

	__reverse(str, str+4);
	TEST_ASSERT_EQUAL_STRING("olleH", str);
}

TEST_GROUP_RUNNER(Util_Test) {
	RUN_TEST_CASE(Util_Test, test_itoa);
	RUN_TEST_CASE(Util_Test, test_reverse);
}