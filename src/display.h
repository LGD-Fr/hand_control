#ifndef CURSES_DISPLAY
#define CURSES_DISPLAY

#include <ncurses.h>
#include <string>

class Curses
{
  private:
    static const int cmd_kbd_lines;
    static const int cmd_kbd_columns;
    WINDOW* cmd_kbd;
    void print_cmd_kbd();

    static const int cmd_speed_lines;
    static const int cmd_speed_columns;
    WINDOW* cmd_speed;
    void print_cmd_speed();

    static const int get_lines;
    static const int get_columns;
    WINDOW* get;

    static const int log_sent_w_lines;
    static const int log_sent_w_columns;
    int log_line_number;
    WINDOW* log_sent_w;

    static const int nav_data_lines;
    static const int nav_data_columns;
    WINDOW* nav_data;
  
  public:
    Curses();
    ~Curses();
    char getchar();

    // TODO
    void update_cmd_speed(const char& coord, const float& v);
    void update_navdata(const float& batteryPercent,
                        const int& state,
                        const float& time);
    void log_sent(const std::string& str);
};

#endif
