#pragma once

//FLASH memory management. Too much F() and conventional PROGMEM crash the application due to being placed before the code itself
// and thus increasing the address pointers of the latter dramatically. This makes access time larger and throws off important timings
// and, eventually, all the code work flow gets screwed up.

#define PROGMEM_LATE __attribute__(( __section__(".fini2") ))

//Fix for MSVC intellisense
#ifdef _VMICRO_INTELLISENSE

#define PSTR_L(s_l) PSTR(s_l) //Dummy!
#define F_L(s_l) F(s_l) //Dummy!

#else

#define PSTR_L(s_l) (__extension__({static char __c[] __attribute__(( __section__(".fini3") )) = (s_l); &__c[0];}))

#ifdef ARDUINO
#define F_L(s_l) (reinterpret_cast<__FlashStringHelper *>(__extension__({static char __c[] __attribute__(( __section__(".fini1") )) = (s_l); &__c[0];})))
#endif

#endif // _VMICRO_INTELLISENSE

