#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

template <typename T, size_t size>
class RingBuffer {
public:
    bool put(T const element) {
        size_t _head = head.load();

        if (_head - tail.load() == size) {
            return false;
        }

        buffer[_head++ & idx_mask] = element;
        head.store(_head);
        return true;
    }

    bool get(T* out) {
        size_t _tail = tail.load();

        if (_tail == head.load()) {
            return false;
        }

        *out = buffer[_tail++ & idx_mask];
        tail.store(_tail);
        return true;
    }

    bool empty() {
        return tail == head;
    }

    void reset() {
        tail = head = 0;
    }

private:
    constexpr static size_t idx_mask = size - 1;

    T buffer[size];
    std::atomic<size_t> head;
    std::atomic<size_t> tail;

    static_assert((size != 0), "buffer cannot be of zero size");
    static_assert((size & idx_mask) == 0, "size is not a power of 2");
};
