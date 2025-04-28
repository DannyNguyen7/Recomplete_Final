#include "timer_software.h"
#include "main.h"

int timer_counter[10] = {0}; // Mảng đếm thời gian cho 10 timer
int timerFLag[10] = {0};    // Mảng cờ cho 10 timer

void set_timer(int index, int counter)
{
    timer_counter[index] = counter / 10;
    timerFLag[index] = 0;
}

void timer_run()
{
    for(int i = 0; i < 10; i++){
      if(timer_counter[i] > 0){
        timer_counter[i]--;
        if(timer_counter[i] <= 0){
          timerFLag[i] = 1;
        }
      }
    }
}