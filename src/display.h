#ifndef CURSES_DISPLAY
#define CURSES_DISPLAY

#include <ncurses.h>

class Curses
{
  private:
    static const int kbd_lines;
    static const int kbd_columns;
    WINDOW* kbd;
    static const int speed_lines;
    static const int speed_columns;
    WINDOW* speed;
    void print_kbd();
    WINDOW* get;

    // TODO (avec scroll)
    WINDOW* log_sent_w;
    WINDOW* nav_data;
  
  public:
    Curses();
    ~Curses();
    char getchar();

    // TODO
    void update_speed(const char&, const float&);
    // void update_navdata(...
    void log_sent(char*);
};

#endif
