#pragma once

#include <cstdint>

struct Timestamp {
  static constexpr int MAX_NSEC_VALUE = 1000000000;

  uint32_t sec;
  uint32_t nsec;

  Timestamp(uint32_t secValue, uint32_t nsecValue)
      : sec(secValue), nsec(nsecValue) {
    if (nsec > MAX_NSEC_VALUE) {
      sec += nsec % MAX_NSEC_VALUE;
      nsec = nsec / MAX_NSEC_VALUE;
    }
  }

  Timestamp() = default;
  ~Timestamp() = default;
  Timestamp(Timestamp &another) = default;
  Timestamp(Timestamp &&another) = default;
  Timestamp &operator=(const Timestamp &another) = default;
  Timestamp &operator=(Timestamp &&another) = default;

  bool operator!=(const Timestamp &another) const {
    return sec != another.sec || nsec != another.nsec;
  }

  bool operator==(const Timestamp &another) const {
    return sec == another.sec && nsec == another.nsec;
  }

  bool operator<(const Timestamp &another) const {
    return sec == another.sec ? nsec < another.nsec : sec < another.sec;
  }

  bool operator>(const Timestamp &another) const {
    return sec == another.sec ? nsec > another.nsec : sec > another.sec;
  }

  bool operator<=(const Timestamp &another) const {
    return sec == another.sec ? nsec <= another.nsec : sec <= another.sec;
  }

  bool operator>=(const Timestamp &another) const {
    return sec == another.sec ? nsec >= another.nsec : sec >= another.sec;
  }

  double toSec() const { return sec + nsec * 1e-9; };

  double toMSec() const { return sec * 1e3 + nsec * 1e-6; };

  double toUSec() const { return sec * 1e6 + nsec * 1e-3; };

  double toNSec() const { return sec * 1e9 + nsec; };
};