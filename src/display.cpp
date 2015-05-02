#include <ncurses.h>
#include <string>
#include "display.h"

const int Curses::cmd_kbd_lines = 12;
const int Curses::cmd_kbd_columns = 50;

const int Curses::cmd_speed_lines = 4;
const int Curses::cmd_speed_columns = 50;

const int Curses::get_lines = 1;
const int Curses::get_columns = 1;

const int Curses::nav_data_lines = 5;
const int Curses::nav_data_columns = 40;

const int Curses::log_sent_w_lines = 12;
const int Curses::log_sent_w_columns = 40;

Curses::Curses() {
  initscr();
  cbreak();
  //noecho();

  cmd_kbd = newwin(cmd_kbd_lines, cmd_kbd_columns, 0, 0);

  get = newwin(get_lines, get_columns,
               cmd_kbd_lines, cmd_kbd_columns/2);

  cmd_speed = newwin(cmd_speed_lines, cmd_speed_columns,
                cmd_kbd_lines + get_lines, 0);

  log_sent_w = newwin(log_sent_w_lines, log_sent_w_columns,
                      0, cmd_kbd_columns + 1);
  log_line_number = log_sent_w_lines - 1;
  wattron(log_sent_w, A_BOLD);
  start_color();
  init_pair(1, COLOR_RED, COLOR_BLACK);
  wattron(log_sent_w, COLOR_PAIR(1));

  scrollok(log_sent_w, TRUE);

  nav_data = newwin(nav_data_lines, nav_data_columns,
                    log_sent_w_lines + 1, cmd_kbd_columns + 1);

  print_cmd_kbd();
  print_cmd_speed();

  wmove(get, 0, 0);
  wrefresh(get);
}

Curses::~Curses() {
  delwin(cmd_kbd);
  delwin(cmd_speed);
  delwin(log_sent_w);
  delwin(nav_data);
  delwin(get);
  endwin();
}

char Curses::getchar() {
  return wgetch(get);
}

void Curses::print_cmd_kbd() {
  wmove(cmd_kbd, 0, 0); waddstr(cmd_kbd, "        ---------------------");
  wmove(cmd_kbd, 1, 0); waddstr(cmd_kbd, "takeoff>|  t|⇑ y|↖ u|↑ i|↗ o|");
  wmove(cmd_kbd, 2, 0); waddstr(cmd_kbd, "        |---|---|---|---|---|----");
  wmove(cmd_kbd, 3, 0); waddstr(cmd_kbd, "  reset>|  g|⇐ h|← j|  k|→ l|⇒ m|");
  wmove(cmd_kbd, 4, 0); waddstr(cmd_kbd, "        |---|---|---|---|---|----");
  wmove(cmd_kbd, 5, 0); waddstr(cmd_kbd, "   land>|  b|⇓ n|↙ ,|↓ ;|↘ :|");
  wmove(cmd_kbd, 6, 0); waddstr(cmd_kbd, "        ---------------------");
  wmove(cmd_kbd, 8, 0); waddstr(cmd_kbd, "a/w : increase/decrease linear `x` speeds by 10%");
  wmove(cmd_kbd, 9, 0); waddstr(cmd_kbd, "z/x : increase/decrease linear `y` speed by 10%");
  wmove(cmd_kbd, 10, 0); waddstr(cmd_kbd, "e/c : increase/decrease linear `z` speed by 10%");
  wmove(cmd_kbd, 11, 0); waddstr(cmd_kbd, "r/v : increase/decrease rotation speed by 10%");
  wrefresh(cmd_kbd);
}

void Curses::print_cmd_speed() {

}

void Curses::update_cmd_speed(const char& coord, const float& v) {

}

void Curses::log_sent(const std::string& str) {
  wmove(log_sent_w, log_line_number++, 0);
  waddstr(log_sent_w, (str + "\n").c_str() );
  wrefresh(log_sent_w);
}

void Curses::update_navdata(const float& batteryPercent,
                            const int& state,
                            const float& time) {

}
