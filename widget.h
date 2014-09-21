/*****************************************************************
 * widget.h
 * hello_avr
 *
 *  Created on		: Sep 21, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/

#ifndef WIDGET_H_
#define WIDGET_H_
#include <inttypes.h>
// state machines
struct widget_base {
	struct widget_base * parent;
	// joystick direction
	struct widget_base * up;
	struct widget_base * left;
	struct widget_base * down;
	struct widget_base * right;
	// for enter key action
	void (*action)(void);
};
typedef struct widget_base wbase_t;

// widget canvas, representing a screen's display content
struct widget_canvas {
	wbase_t super;
	// non-overlapping all the items linked by up and down pointer.
	wbase_t *first_item_to_display;
};
typedef struct widget_canvas wcanvas_t;

// widget button
struct widget_button {
	wbase_t super;
	uint8_t position_x;
	uint8_t position_y;
	char * text;// text to display, be careful of the length
};
typedef struct widget_button wbutton_t;
void wbase_init(wbase_t * object,
		wbase_t * parent,
		wbase_t * up,
		wbase_t * left,
		wbase_t * down,
		wbase_t * right,
		void (*action)(void));

void wbutton_init(wbutton_t * object,
		uint8_t position_x,
		uint8_t position_y,
		const char * text,
		wbase_t * parent,
		wbase_t * up,
		wbase_t * left,
		wbase_t * down,
		wbase_t * right,
		void (*action)(void));

void wcanvas_init(wcanvas_t * object,
		wbase_t * parent,
		wbase_t * up,
		wbase_t * left,
		wbase_t * down,
		wbase_t * right,
		void (*action)(void),
		wbase_t * first_to_display);

void wcanvas_display(wcanvas_t * object);
#endif /* WIDGET_H_ */
