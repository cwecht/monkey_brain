#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "monkey_brain_basic_logger/basic_logger.hpp"

namespace mbc = monkey_brain_core;
namespace mbbl = monkey_brain_basic_logger;

class void_value {
  struct void_base
  {
    virtual ~void_base() = default;
    virtual std::unique_ptr<void_base> clone() const = 0;
    virtual void const * get() const = 0;
  };

  template<typename T>
  class void_impl : public void_base {
public:
    template<typename TT>
    void_impl(TT && v)
    : value_{std::forward<TT>(v)} {}

    std::unique_ptr<void_base> clone() const override
    {
      return std::make_unique<void_impl<T>>(value_);
    }

    void const * get() const override {return &value_;}

private:
    T value_;
  };

public:
  void_value() = default;
  ~void_value() = default;
  void_value(void_value &&) = default;
  void_value & operator=(void_value &&) & = default;

  void_value(const void_value & other)
  : pimpl_{clone(other.pimpl_.get())} {}

  void_value & operator=(const void_value & other) &
  {
    if (this != &other) {
      pimpl_ = clone(other.pimpl_.get());
    }
    return *this;
  }

  template<typename T>
  void_value(T && value)
  : pimpl_{std::make_unique<void_impl<T>>(std::forward<T>(value))} {}

  void const * get() const
  {
    assert(pimpl_ != nullptr);
    return pimpl_->get();
  }

private:
  static std::unique_ptr<void_base> clone(void_base const * other)
  {
    return (other == nullptr) ? nullptr : other->clone();
  }
  std::unique_ptr<void_base> pimpl_ = nullptr;
};

using ::testing::Eq;

TEST(BasicLogger, logs_string_without_format_unmodified) {
  const std::string format_string = "an example format string";
  const mbbl::BasicLogger logger{format_string, {}};

  EXPECT_THAT(logger.get_expected_length(), Eq(format_string.size()));

  std::string buffer(logger.get_expected_length(), '\0');
  const auto result = logger.to_chars(buffer.data(), buffer.data() + buffer.size() + 1);
  EXPECT_THAT(result.ec, std::errc{});

  EXPECT_THAT(buffer, Eq(format_string));
}

TEST(BasicLogger, logs_string_with_format_unmodified_and_no_references) {
  const std::string format_string = "an example {} format string";
  const mbbl::BasicLogger logger{format_string, {}};

  EXPECT_THAT(logger.get_expected_length(), Eq(format_string.size()));

  std::string buffer(logger.get_expected_length(), '\0');
  const auto result = logger.to_chars(buffer.data(), buffer.data() + buffer.size() + 1);
  EXPECT_THAT(result.ec, std::errc{});

  EXPECT_THAT(buffer, Eq(format_string));
}

TEST(BasicLogger, logs_format_string_with_string) {
  const std::string format_string = "an example {} format string";
  const mbc::TypedReferences references =
  {{"an_string", mbc::ValueTypes::STRING, mbc::AccessMode::WRITEONLY}};
  mbbl::BasicLogger logger{format_string, references};

  std::string any_string = "some example string";
  logger.assign_value("an_string", &any_string);

  std::string buffer(logger.get_expected_length(), '\0');
  const auto result = logger.to_chars(buffer.data(), buffer.data() + buffer.size() + 1);
  EXPECT_THAT(result.ec, std::errc{});

  EXPECT_THAT(buffer, Eq("an example some example string format string"));
}

struct ValueWithResult
{
  void_value value;
  mbc::ValueType type;
  std::string expected;
};

using BasicLogger_value = ::testing::TestWithParam<ValueWithResult>;

TEST_P(BasicLogger_value, logs_format_string_with_value) {
  const auto & [value, type, expected] = GetParam();
  const std::string format_string = "{}";
  const mbc::TypedReferences references =
  {{"value", type, mbc::AccessMode::WRITEONLY}};
  mbbl::BasicLogger logger{format_string, references};

  logger.assign_value("value", value.get());

  std::string buffer(logger.get_expected_length(), '\0');
  const auto result = logger.to_chars(buffer.data(), buffer.data() + buffer.size() + 1);
  EXPECT_THAT(result.ec, std::errc{});

  EXPECT_THAT(buffer, Eq(expected));
}

INSTANTIATE_TEST_SUITE_P(
   AllOf, BasicLogger_value,
   ::testing::Values(
     ValueWithResult{true, mbc::ValueTypes::BOOL, "true"},
     ValueWithResult{false, mbc::ValueTypes::BOOL, "false"},
     ValueWithResult{int32_t{42}, mbc::ValueTypes::INT32, "42"},
     ValueWithResult{int32_t{-42}, mbc::ValueTypes::INT32, "-42"},
     ValueWithResult{int32_t{2'147'483'647}, mbc::ValueTypes::INT32, "2147483647"},
     ValueWithResult{int32_t{-2'147'483'648}, mbc::ValueTypes::INT32, "-2147483648"},
     ValueWithResult{float{4.2f}, mbc::ValueTypes::FLOAT32, "4.2"},
     ValueWithResult{float{-4.2f}, mbc::ValueTypes::FLOAT32, "-4.2"},
     ValueWithResult{float{std::numeric_limits<float>::quiet_NaN()}, mbc::ValueTypes::FLOAT32,
      "nan"},
     ValueWithResult{double{4.2}, mbc::ValueTypes::FLOAT64, "4.2"},
     ValueWithResult{double{-4.2}, mbc::ValueTypes::FLOAT64, "-4.2"},
     ValueWithResult{double{std::numeric_limits<double>::quiet_NaN()}, mbc::ValueTypes::FLOAT64,
      "nan"},
     ValueWithResult{std::string{"any string"}, mbc::ValueTypes::STRING, "any string"},
     ValueWithResult{std::vector<int32_t>{}, mbc::ValueTypes::INT32_ARRAY, "[]"},
     ValueWithResult{std::vector<int32_t>{5}, mbc::ValueTypes::INT32_ARRAY, "[5]"},
     ValueWithResult{std::vector<int32_t>{5, 6}, mbc::ValueTypes::INT32_ARRAY, "[5, 6]"},
     ValueWithResult{std::vector<std::string>{"first", "second"}, mbc::ValueTypes::STRING_ARRAY,
      "[first, second]"}
));
