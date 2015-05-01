#include "display.h"
#include "locale.h"
int main()
{
setlocale(LC_ALL, "");
Curses term;
term.print_kbd();
for(;;) ;
}

