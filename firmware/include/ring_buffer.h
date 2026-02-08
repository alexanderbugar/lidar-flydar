#pragma once

#include <cstddef>

template<typename T, size_t Capacity>
class RingBuffer {
    static_assert(Capacity > 0, "a ring needs slots");
public:
    void push(const T& value) {
        data_[next_] = value;
        next_ = (next_ + 1) % Capacity;
        if (size_ < Capacity) ++size_;
    }

    size_t size() const { return size_; }

    // offset zero is the newest. caller owns the ring and checks size().
    const T& newest(size_t offset = 0) const {
        return data_[(next_ + Capacity - 1 - offset) % Capacity];
    }

private:
    T data_[Capacity]{};
    size_t next_ = 0;
    size_t size_ = 0;
};
