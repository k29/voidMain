#include <ncurses.h>


int main()
{

	WINDOW *m;
	m = initscr();
	
	
	printw("hi\n");
	printw("lol\n");
	refresh();
	getch();
	endwin();
	
}
