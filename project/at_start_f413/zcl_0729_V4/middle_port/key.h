#ifndef __KEY
#define __KEY


extern void ButtonInit(void);
extern unsigned char GetButtonEvent(unsigned char buttonId);
extern unsigned char ButtonScan(unsigned int timeMs);

#endif
