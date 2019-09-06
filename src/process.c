/************** uart.h *******************
 * AUTHOR:			Simon Lindhorst
 * CREATE:			03.03.2018
 * LAST_CHANGE:	10.03.2018
 * NOTE:				This code was written for ATxMega8E5, ATxMega16E5, ATxMega32E5
 * PROBLEMS:		Nothing known
 * TODO:				
 */

#include "process.h"
#include <stdio.h>
#include <avr/interrupt.h>

//List of all processes

//_proc_hdl_list and _proc_hdl_buffer must have the same size!
process_hdl_t *_proc_hdl_list[PROCESS_MAX_AMOUNT];
process_hdl_t _proc_hdl_buffer[PROCESS_MAX_AMOUNT];

#if PROCESS_MAX_AMOUNT > 8
	#error "PROCESS_MAX_AMOUNT only less equal 8 supported"
#endif

void process_order_by_prior(void);
uint8_t process_get_id();

process_prior_t some_processes_wants_to_run;

void process_init(){
	uint8_t i;
	
	for(i=0; i<PROCESS_MAX_AMOUNT; i++){
		_proc_hdl_buffer[i].prior = PROCESS_PRIOR_UNUSED;
		_proc_hdl_list[i] = NULL;
	}
	
	some_processes_wants_to_run = PROCESS_PRIOR_UNUSED;
}

uint8_t process_get_id(){
	uint8_t i;
	
	for(i=0; i<PROCESS_MAX_AMOUNT; i++){
		if(_proc_hdl_buffer[i].prior == PROCESS_PRIOR_UNUSED)
			return i+1;
	}
	
	return 0;
}

process_hdl_t *process_add(process_prior_t prior, uint16_t call_time_ms, \
		uint8_t (*callback_hdl)(process_hdl_t *))
{
	process_hdl_t* p_hdl;
	uint8_t i;
	
	i = process_get_id();
	
	if(i){
		p_hdl = &(_proc_hdl_buffer[i-1]);
		p_hdl->function_hdl = callback_hdl;
		p_hdl->counter_max = call_time_ms;
		p_hdl->_counter = call_time_ms;
		p_hdl->id = i;
		p_hdl->prior = prior;
		p_hdl->wants_to_run = 0;
		
		//_proc_hdl_list and _proc_hdl_buffer must have the same size!
		for(i=0; i<PROCESS_MAX_AMOUNT && (_proc_hdl_list[i]); i++);
		cli();
		_proc_hdl_list[i] = p_hdl;
		sei();
		
		return p_hdl;
	}
	return NULL;
}

/*
void process_order_by_prior(){
	process_hdl_t *p_hdl;
	uint8_t i,a,idx;
	
	idx = 0;
	for(i=PROCESS_PRIOR_HIGH; i<PROCESS_PRIOR_UNUSED; i++){
		for(a=0; a<PROCESS_MAX_AMOUNT; a++){
			p_hdl = _proc_hdl_buffer+a;
			if(p_hdl->prior == i){
				(_proc_hdl_list+idx) = p_hdl;
				idx++;
			}
		}
	}
}*/

//This Funktion should only called by Interrupt!!!
process_prior_t process_update_counter_all(void){
	uint8_t i;
	process_hdl_t *p_hdl;
	
	p_hdl = _proc_hdl_list[0];
	i=0;
	while(p_hdl && i<PROCESS_MAX_AMOUNT){
		if(p_hdl->_counter > 1)
			(p_hdl->_counter)--;
		else if(p_hdl->prior < PROCESS_PRIOR_UNUSED){
			p_hdl->wants_to_run = 1;
			if(p_hdl->prior < some_processes_wants_to_run)
				some_processes_wants_to_run = p_hdl->prior;
			p_hdl->_counter = p_hdl->counter_max;	//Reset counter for timeout
		}
		i++;
		p_hdl++;
	}
	
	return some_processes_wants_to_run;
}

uint8_t process_delete(process_hdl_t *p_hdl){
	int8_t i;
	uint8_t idx_last=PROCESS_MAX_AMOUNT;
	
	if(!p_hdl)
		return 1;
	
	for(i=PROCESS_MAX_AMOUNT-1; i>=0; i--){
		if(idx_last >= PROCESS_MAX_AMOUNT && _proc_hdl_list[i])
			idx_last = i;
		if(_proc_hdl_list[i] == p_hdl){
			if(i == idx_last){
				cli();
				_proc_hdl_list[i] = NULL;
				sei();
			}else{
				cli();
				_proc_hdl_list[i] = _proc_hdl_list[idx_last];
				_proc_hdl_list[idx_last] = NULL;
				sei();
			}
			return 0;
		}
	}
	return 1;
}

process_hdl_t *process_get_timouted_hdl(void){
	uint8_t i;
	process_hdl_t *p_hdl;
	process_hdl_t *p_most_prior = NULL;
	
	p_hdl = _proc_hdl_list[0];
	i=0;
	while(p_hdl && i<PROCESS_MAX_AMOUNT){
		if(p_hdl->wants_to_run){
			if(p_hdl->prior == 0){
				p_most_prior = p_hdl;
				break;
			}else if(!p_most_prior || p_hdl->prior < p_most_prior->prior){
				p_most_prior = p_hdl;
			}
		}
		i++;
		p_hdl = _proc_hdl_list[i];
	}
	
	if(p_most_prior){
		p_most_prior->wants_to_run = 0;
	}
	
	return p_most_prior;
}
