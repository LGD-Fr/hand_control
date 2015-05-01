#ifndef CURSES
#define CURSES

#include <ncurses.h>

class Curses
{
  private:
    static const int kbd_lines;
    static const int kbd_columns;
    WINDOW* kbd;
    static const char** kbd_descr;
    static const int speed_lines;
    static const int speed_columns;
    WINDOW* speed;
  
  public:
    Curses();
    ~Curses();
    void print_kbd();
};

#endif
