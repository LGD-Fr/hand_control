#include <ncurses.h>
#include "display.h"

const int Curses::kbd_lines = 12;
const int Curses::kbd_columns = 50;
const int Curses::speed_lines = 4;
const int Curses::speed_columns = 50;

Curses::Curses() {
  initscr();
  cbreak();
  noecho();
  kbd = newwin(kbd_lines, kbd_columns, 0, 0);
  speed = newwin(speed_lines, speed_columns,
               kbd_lines+1, 0);
}

Curses::~Curses() {
  delwin(kbd);
  delwin(speed);
  endwin();
}

void Curses::print_kbd() {
  wmove(kbd, 0, 0); waddstr(kbd, "        ---------------------");
  wmove(kbd, 1, 0); waddstr(kbd, "takeoff>|  t|⇑ y|↖ u|↑ i|↗ o|");
  wmove(kbd, 2, 0); waddstr(kbd, "        |---|---|---|---|---|----");
  wmove(kbd, 3, 0); waddstr(kbd, "  reset>|  g|⇐ h|← j|  k|→ l|⇒ m|");
  wmove(kbd, 4, 0); waddstr(kbd, "        |---|---|---|---|---|----");
  wmove(kbd, 5, 0); waddstr(kbd, "   land>|  b|⇓ n|↙ ,|↓ ;|↘ :|");
  wmove(kbd, 6, 0); waddstr(kbd, "        ---------------------");
  wmove(kbd, 8, 0); waddstr(kbd, "a/w : increase/decrease linear `x` speeds by 10%");
  wmove(kbd, 9, 0); waddstr(kbd, "z/x : increase/decrease linear `y` speed by 10%");
  wmove(kbd, 10, 0); waddstr(kbd, "e/c : increase/decrease linear `z` speed by 10%");
  wmove(kbd, 11, 0); waddstr(kbd, "r/v : increase/decrease rotation speed by 10%");
  wrefresh(kbd);
}
