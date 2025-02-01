#include "logger.h"

Stream *LOGGER::_stream = nullptr;
bool LOGGER::_enable = true;

void LOGGER::begin(Stream *s) {
    if (s != nullptr) {
        _stream = s;
    }
}

void LOGGER::enable(bool enable) {
    _enable = enable;
}

void LOGGER::print(const String &s) {
    if (_enable && _stream != nullptr) {
        _stream->print(s);
    }
}

void LOGGER::print(const char &c) {
    if (_enable && _stream != nullptr) {
        _stream->print(c,HEX);
    }
}


void LOGGER::println(const String &s) {
    if (_enable && _stream != nullptr) {
        _stream->println(s);
    }
}
