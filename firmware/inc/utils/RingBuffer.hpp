#pragma once

#include <cstddef>

template <typename T, size_t size>
class RingBuffer {
public:
    bool put(T const element) {
        size_t _head = (head.load() + 1) % size;

        if (_head == tail.load()) {
            return false;
        }

        buffer[_head] = element;
        this.head.store(_head);
        return true;
    }

    bool get(T* out) {
        size_t _tail = tail.load();

        if (head.load() == tail) {
            return false;
        }

        *out = buffer[tail];
        tail.store((tail + 1) % size);
        return true;
    }

private:
    T buffer[size];
    sts::atomic<size_t> head;
    sts::atomic<size_t> tail;
};
