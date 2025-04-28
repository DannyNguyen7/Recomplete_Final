#ifndef INC_TIMER_SOFTWARE_H_
#define INC_TIMER_SOFTWARE_H_

extern int timer_counter[10];
extern int timerFLag[10];

void timer_run();
void set_timer(int index, int counter);


#endif