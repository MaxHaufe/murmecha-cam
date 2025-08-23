//
// Created by max on 22/08/25.
//

#ifndef RINGBUFFER_HPP
#define RINGBUFFER_HPP

#include <vector>
#include <stdexcept>

template<typename T>
class RingBuffer {
private:
    std::vector<T> buffer;
    size_t head; // Points to the front element
    size_t tail; // Points to where next element will be inserted
    size_t count; // Current number of elements
    size_t capacity;

public:
    explicit RingBuffer(size_t n) : buffer(n), head(0), tail(0), count(0), capacity(n) {
        if (n == 0) throw std::invalid_argument("Queue size must be > 0");
    }

    // add element (overwrites oldest if full)
    void push(const T &item) {
        buffer[tail] = item;
        tail = (tail + 1) % capacity;

        if (count == capacity) {
            head = (head + 1) % capacity; // overwrite oldest
        } else {
            count++;
        }
    }

    // rm front element
    void pop() {
        if (empty()) throw std::runtime_error("Queue is empty");
        head = (head + 1) % capacity;
        count--;
    }

    // front element
    T &front() {
        if (empty()) throw std::runtime_error("Queue is empty");
        return buffer[head];
    }

    const T &front() const {
        if (empty()) throw std::runtime_error("Queue is empty");
        return buffer[head];
    }

    //  back element
    T &back() {
        if (empty()) throw std::runtime_error("Queue is empty");
        return buffer[(tail - 1 + capacity) % capacity];
    }

    const T &back() const {
        if (empty()) throw std::runtime_error("Queue is empty");
        return buffer[(tail - 1 + capacity) % capacity];
    }

    // check if empty
    bool empty() const { return count == 0; }

    // check if full
    bool full() const { return count == capacity; }

    // current size
    size_t size() const { return count; }

    // max capacity
    size_t max_size() const { return capacity; }

    // clear all elements
    void clear() {
        head = tail = count = 0;
    }

    // Get all elements in order (oldest to newest)
    std::vector<T> get() const {
        std::vector<T> result;
        result.reserve(count);

        for (size_t i = 0; i < count; ++i) {
            result.push_back(buffer[(head + i) % capacity]);
        }

        return result;
    }
};

#endif //RINGBUFFER_HPP
