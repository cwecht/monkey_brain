#ifndef MONKEY_BRAIN_STATE_NAME_RANGE_HPP
#define MONKEY_BRAIN_STATE_NAME_RANGE_HPP

#include "monkey_brain_scxml/states.hpp"

#include <boost/container/flat_set.hpp>

#include <string_view>
#include <vector>

namespace monkey_brain_scxml
{

class StateNameIterator
{
public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = std::string_view;
  using difference_type = std::ptrdiff_t;
  using pointer = std::string_view *;
  using reference = std::string_view;

  explicit StateNameIterator(State const * const * it)
  : iterator_{it} {}

  std::string_view operator*() const {return (*iterator_)->name;}

  StateNameIterator & operator++() {++iterator_; return *this;}

  friend bool operator==(const StateNameIterator & lhs, const StateNameIterator & rhs)
  {
    return lhs.iterator_ == rhs.iterator_;
  }

  friend bool operator!=(const StateNameIterator & lhs, const StateNameIterator & rhs)
  {
    return lhs.iterator_ != rhs.iterator_;
  }

  StateNameIterator operator+(std::size_t offset) const
  {
    return StateNameIterator{iterator_ + offset};
  }

  StateNameIterator operator++(int)
  {
    StateNameIterator old = *this;
    operator++();
    return old;
  }

private:
  State const * const * iterator_;
};

class StateNameRange
{
public:
  using value_type = std::string_view;
  using const_iterator = StateNameIterator;

  template<typename Range>
  explicit StateNameRange(const Range & states)
  : begin_{&(*states.begin())}
    , end_{begin_ + std::distance(states.begin(), states.end())} {}

  const_iterator begin() const {return begin_;}
  const_iterator end() const {return end_;}

private:
  const_iterator begin_;
  const_iterator end_;
};

} // namespace monkey_brain_scxml
#endif // MONKEY_BRAIN_STATE_NAME_RANGE_HPP
