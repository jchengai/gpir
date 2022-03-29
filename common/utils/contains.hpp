// https://codereview.stackexchange.com/questions/59997/contains-algorithm-for-stdvector/59999

#include <algorithm>

#pragma once

namespace std_enhancement {

template <class C, class T>
inline auto contains_impl(const C& c, const T& x, int)
    -> decltype(c.find(x), true) {
  return end(c) != c.find(x);
}

template <class C, class T>
inline bool contains_impl(const C& v, const T& x, long) {
  return end(v) != std::find(begin(v), end(v), x);
}

template <class C, class T>
auto contains(const C& c, const T& x) -> decltype(end(c), true) {
  return contains_impl(c, x, 0);
}

}  // namespace std_enhancement