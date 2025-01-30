#ifndef _APP_LOGGER_H_
#define _APP_LOGGER_H_

#include "Arduino.h"

//日志类
 class LOGGER
{
    public:
    static void begin(Stream *s);
    static void enable(bool en);
    static void print(const String &s);
    static void println(const String &s);
    static void print(const char &c);

    private:
     static Stream *_stream ;
     static bool _enable;
        
 };

#endif