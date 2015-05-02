#include <ncurses.h>
#include <string>
#include "display.h"

const int Curses::cmd_kbd_lines = 7;
const int Curses::cmd_kbd_columns = 55;

const int Curses::cmd_speed_lines = 4;
const int Curses::cmd_speed_columns = 55;

const int Curses::get_lines = 1;
const int Curses::get_columns = 1;

const int Curses::nav_data_lines = 3;
const int Curses::nav_data_columns = 55;

const int Curses::log_sent_w_lines = 12;
const int Curses::log_sent_w_columns = 40;

Curses::Curses() {
  initscr();
  cbreak();
  start_color();
  //noecho();

  cmd_kbd = newwin(cmd_kbd_lines, cmd_kbd_columns, 0, 0);

  get = newwin(get_lines, get_columns,
               cmd_kbd_lines, cmd_kbd_columns/2);

  cmd_speed = newwin(cmd_speed_lines, cmd_speed_columns,
                cmd_kbd_lines + get_lines, 0);

  log_sent_title = newwin(1, log_sent_w_columns,
                          0, cmd_kbd_columns + 1);
  waddstr(log_sent_title, "SENT COMMANDS");
  wrefresh(log_sent_title);
  log_sent_w = newwin(log_sent_w_lines - 1, log_sent_w_columns,
                      1, cmd_kbd_columns + 1);
  log_line_number = log_sent_w_lines - 2; 
  wattron(log_sent_w, A_BOLD);
  init_pair(1, COLOR_RED, COLOR_BLACK);
  wattron(log_sent_w, COLOR_PAIR(1));
  scrollok(log_sent_w, TRUE);

  nav_data = newwin(nav_data_lines, nav_data_columns,
                    cmd_kbd_lines + get_lines + cmd_speed_lines + 1, 0);

  print_nav_data();
  print_cmd_kbd();
  print_cmd_speed();

  wmove(get, 0, 0);
  wrefresh(get);
}

Curses::~Curses() {
  delwin(cmd_kbd);
  delwin(cmd_speed);
  delwin(log_sent_w);
  delwin(log_sent_title);
  delwin(nav_data);
  delwin(get);
  endwin();
}

char Curses::getchar() {
  return wgetch(get);
}

void Curses::print_cmd_kbd() {
  wmove(cmd_kbd, 0, 0);
   waddstr(cmd_kbd, "        ---------------------\n");
   waddstr(cmd_kbd, "takeoff>|  t|⇑ y|↖ u|↑ i|↗ o|\n");
   waddstr(cmd_kbd, "        |---|---|---|---|---|----\n");
   waddstr(cmd_kbd, "  reset>|  g|⇐ h|← j|  k|→ l|⇒ m|\n");
   waddstr(cmd_kbd, "        |---|---|---|---|---|----\n");
   waddstr(cmd_kbd, "   land>|  b|⇓ n|↙ ,|↓ ;|↘ :|\n");
   waddstr(cmd_kbd, "        ---------------------\n");
  wrefresh(cmd_kbd);
}

void Curses::print_cmd_speed() {
  wmove(cmd_speed, 0, 0);
  waddstr(cmd_speed, " `x` cmd speed :         (a/w : increase/decrease)\n");
  waddstr(cmd_speed, " `y` cmd speed :         (z/x : increase/decrease)\n");
  waddstr(cmd_speed, " `z` cmd speed :         (e/c : increase/decrease)\n");
  waddstr(cmd_speed, "rotation speed :         (r/v : increase/decrease)\n");
  wrefresh(cmd_speed);
}

void Curses::update_cmd_speed(const char& coord, const float& v) {
  switch(coord) {
    case 'x':
      wmove(cmd_speed, 0, 16);
      wprintw(cmd_speed, "%f", v);
      break;
    case 'y':
      wmove(cmd_speed, 1, 16);
      wprintw(cmd_speed, "%f", v);
      break;
    case 'z':
      wmove(cmd_speed, 2, 16);
      wprintw(cmd_speed, "%f", v);
      break;
    case 't':
      wmove(cmd_speed, 3, 16);
      wprintw(cmd_speed, "%f", v);
      break;
    default:
      ;
  }
  wrefresh(cmd_speed);
}

void Curses::log_sent(const std::string& str) {
  wmove(log_sent_w, log_line_number++, 0);
  waddstr(log_sent_w, (str + "\n").c_str() );
  wrefresh(log_sent_w);
}


void Curses::print_nav_data() {
  wmove(nav_data, 0, 0);
  waddstr(nav_data, "Battery :\n  State :\n   Time :");
  wrefresh(nav_data);
}

void Curses::update_navdata(const float& batteryPercent,
                            const int& state,
                            const float& time) {
}
