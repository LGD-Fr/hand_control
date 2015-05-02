#include <ncurses.h>
#include <string>
#include "display.h"

const int Curses::cmd_kbd_lines = 12;
const int Curses::cmd_kbd_columns = 50;
const int Curses::cmd_speed_lines = 4;
const int Curses::cmd_speed_columns = 50;

Curses::Curses() {
  initscr();
  cbreak();
  //noecho();
  cmd_kbd = newwin(cmd_kbd_lines, cmd_kbd_columns, 0, 0);
  cmd_speed = newwin(cmd_speed_lines,
                    cmd_speed_columns, cmd_kbd_lines+1, 0);
  get = newwin(1,1,cmd_kbd_lines + cmd_speed_lines + 1,2);
  //log_sent_w = newwin(///
  //nav_data = newwin(//
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

}

void Curses::update_navdata(const float& batteryPercent,
                            const int& state,
                            const float& time) {

}
