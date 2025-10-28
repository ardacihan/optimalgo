#ifndef INPUT_H
#define INPUT_H
#include <any>

class Input {
private:
    std::any data_;
public:
    Input() = default;

    template<typename T>
    explicit Input(T&& value) : data_(std::forward<T>(value)) {}

    template<typename T>
    T get() const {
        return std::any_cast<T>(data_);
    }

    template<typename T>
    bool holds_type() const {
        return data_.type() == typeid(T);
    }

    bool has_value() const {
        return data_.has_value();
    }
};

#endif
