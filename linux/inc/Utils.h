#ifndef UTILS_H__
#define UTILS_H__

unsigned long millis();
void delay(int sec);
void delayMicroseconds(unsigned int us);
void pinMode(uint8_t pin, uint8_t pinMode);
void digitalWrite(uint8_t pin, uint8_t pinMode);

#endif // UTILS_H__