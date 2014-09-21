/*****************************************************************
 * widget.c
 * hello_avr
 *
 *  Created on		: Sep 21, 2014 
 *  Author			: yulongb
 *	Email			: yulongb@stud.ntnu.no
 *  Description		:
 *****************************************************************/
#include "widget.h"

void wbase_init(wbase_t * object,
		wbase_t * parent,
		wbase_t * up,
		wbase_t * left,
		wbase_t * down,
		wbase_t * right,
		void (*action)(void)){
	object->parent = parent;
	object->up = up;
	object->left = left;
	object->down = down;
	object->right = right;
	object->action = action;
}

void wbutton_init(wbutton_t * object,
		uint8_t position_x,
		uint8_t position_y,
		const char * text,
		wbase_t * parent,
		wbase_t * up,
		wbase_t * left,
		wbase_t * down,
		wbase_t * right,
		void (*action)(void)){
	wbase_init( (wbase_t *)object,
			parent,
			up,
			left,
			down,
			right,
			action);

	object->position_x = position_x;
	object->position_y = position_y;
	object->text = text;
}

void wcanvas_init(wcanvas_t * object,
		wbase_t * parent,
		wbase_t * up,
		wbase_t * left,
		wbase_t * down,
		wbase_t * right,
		void (*action)(void),
		wbase_t * first_to_display){
	wbase_init( (wbase_t *)object,
			parent,
			up,
			left,
			down,
			right,
			action);
	object->first_item_to_display = first_to_display;
}

void wcanvas_display(wcanvas_t * object){

}
