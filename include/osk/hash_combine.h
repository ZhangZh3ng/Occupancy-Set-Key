#pragma once

#include <utility>
#include <functional>

// Template function to combine multiple hash values
template <typename T>
void hash_combine(std::size_t& seed, const T& val) {
  seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// Recursive template function to combine multiple hash values
template <typename T, typename... Types>
void hash_combine(std::size_t& seed, const T& val, const Types&... args) {
  hash_combine(seed, val);
  hash_combine(seed, args...);
}

// XOR with a prime number, it can help ensure that common bits are spread out
// and reduce the chance of collisions
std::size_t hash_mix(std::size_t hash_value) {
  return hash_value ^ 0x9e3779b9;
}