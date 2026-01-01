#ifndef Debug_h

#define Debug_h

//#undef serialdebug 
#define serialdebug true

#ifdef serialdebug
#define SERIALPRINT Serial.flush();Serial.print
#define SERIALPRINTLN Serial.flush();Serial.println
#else
#define SERIALPRINT //Serial.flush();Serial.print
#define SERIALPRINTLN //Serial.flush();Serial.println
#endif

#endif
