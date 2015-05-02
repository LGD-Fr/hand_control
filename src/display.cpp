#include <ncurses.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include "display.h"

const int Curses::cmd_kbd_lines = 8;
const int Curses::cmd_kbd_columns = 55;

const int Curses::cmd_speed_lines = 4;
const int Curses::cmd_speed_columns = 55;

const int Curses::get_lines = 1;
const int Curses::get_columns = 1;

const int Curses::nav_data_lines = 3;
const int Curses::nav_data_columns = 25;

const int Curses::log_sent_w_lines = 12;
const int Curses::log_sent_w_columns = 22;

const int Curses::topic_lines = 8;
const int Curses::topic_columns = 22;

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
  init_pair(1, COLOR_GREEN, COLOR_BLACK);
  wattron(log_sent_w, COLOR_PAIR(1));
  scrollok(log_sent_w, TRUE);

  nav_data = newwin(nav_data_lines, nav_data_columns,
                    cmd_kbd_lines + get_lines + cmd_speed_lines + 1, 0);
  init_pair(2, COLOR_RED, COLOR_BLACK);
  wattron(nav_data, COLOR_PAIR(2));
  wattron(nav_data, A_BOLD);

  topic = newwin(topic_lines, topic_columns,
      cmd_kbd_lines + get_lines + cmd_speed_lines + 1,
      nav_data_columns + 1);
  init_pair(3, COLOR_BLUE, COLOR_BLACK);
  wattron(topic, COLOR_PAIR(3));
  wattron(topic, A_BOLD);

  print_nav_data();
  print_cmd_kbd();
  print_cmd_speed();
  print_topic();

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
  delwin(topic);
  endwin();
}

char Curses::getchar() {
  return wgetch(get);
}

void Curses::print_cmd_kbd() {
  wmove(cmd_kbd, 0, 0);
   waddstr(cmd_kbd, "         ---------------------\n");
   waddstr(cmd_kbd, " takeoff>|  t|⇑ y|↖ u|↑ i|↗ o|\n");
   waddstr(cmd_kbd, "         |---|---|---|---|---|----\n");
   waddstr(cmd_kbd, "   reset>|  g|⇐ h|← j|  k|→ l|⇒ m|\n");
   waddstr(cmd_kbd, "         |---|---|---|---|---|----\n");
   waddstr(cmd_kbd, "    land>|  b|⇓ n|↙ ,|↓ ;|↘ :|\n");
   waddstr(cmd_kbd, "         ---------------------\n");
   waddstr(cmd_kbd, "        Press ctrl + C to quit");
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

void Curses::update_nav_data(const float& batteryPercent,
                            const int& state,
                            const float& time) {
  wmove(nav_data, 0, 10);
  wprintw(nav_data, "%f %%", batteryPercent);
  wmove(nav_data, 2, 10);
  wprintw(nav_data, "%f %", time);
  wmove(nav_data, 1, 10);
  switch(state) {
    case 0:
      waddstr(nav_data, "unknown    ");
      break;
    case 1:
      waddstr(nav_data, "inited     ");
      break;
    case 2:
      waddstr(nav_data, "landed     ");
      break;
    case 3:
      waddstr(nav_data, "flying     ");
      break;
    case 4:
      waddstr(nav_data, "hovering   ");
      break;
    case 5:
      waddstr(nav_data, "test       ");
      break;
    case 6:
      waddstr(nav_data, "taking off ");
      break;
    case 7:
      waddstr(nav_data, "flying     ");
      break;
    case 8:
      waddstr(nav_data, "landing    ");
      break;
    case 9:
      waddstr(nav_data, "looping    ");
      break;
    default:
      ;
  }
  wrefresh(nav_data);
}

void Curses::print_topic() {
  wmove(topic, 0, 0);
  waddstr(topic, "Linear :\n x : \n y : \n z : \n");
  waddstr(topic, "Angular :\n x : \n y : \n z : ");
  wrefresh(topic);
}

void Curses::update_topic(const geometry_msgs::Twist::ConstPtr& twist) {
  wmove(topic, 1, 5); wprintw(topic, "%f  ", twist->linear.x);
  wmove(topic, 2, 5); wprintw(topic, "%f  ", twist->linear.y);
  wmove(topic, 3, 5); wprintw(topic, "%f  ", twist->linear.z);
  wmove(topic, 5, 5); wprintw(topic, "%f  ", twist->angular.x);
  wmove(topic, 6, 5); wprintw(topic, "%f  ", twist->angular.y);
  wmove(topic, 7, 5); wprintw(topic, "%f  ", twist->angular.z);
  wrefresh(topic);
}
