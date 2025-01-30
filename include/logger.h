#ifndef _LOGGER_LIB_H_
#define _LOGGER_LIB_H_

#include "Arduino.h"

 class LOGGER
{
    public:
    static void begin(Stream *s);
    static void enable(bool en);
    static void print(const String &s);
    static void println(const String &s);
    

    private:
     static Stream *_stream ;
     static bool _enable;
        
 };

#endif