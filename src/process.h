/************** process.h *******************
 * AUTHOR:			Simon Lindhorst
 * CREATE:			03.03.2018
 * LAST_CHANGE:	10.03.2018
 * NOTE:				
 * PROBLEMS:		Nothing known
 * TODO:				
 */
#ifndef PROCESS_H
#define PROCESS_H

#include <stdint.h>

#define PROCESS_MAX_AMOUNT	8

//from important to absolutly unimportant
enum{
	PROCESS_PRIOR_HIGH = 0,
	PROCESS_PRIOR_MED ,
	PROCESS_PRIOR_LOW,
	PROCESS_PRIOR_PETTY,
	PROCESS_PRIOR_UNUSED	//Never use that priority!!!
};

typedef struct st_process_hdl{
	volatile uint16_t _counter;
	uint8_t (*function_hdl)(struct st_process_hdl *);
	uint16_t counter_max;
	uint8_t id;
	uint8_t prior;
	volatile uint8_t wants_to_run;
}process_hdl_t;

typedef uint8_t process_prior_t;

void process_init(void);

process_hdl_t *process_add(process_prior_t prior, uint16_t call_time_ms, uint8_t (*callback_hdl)(process_hdl_t*));

process_prior_t process_update_counter_all(void);

process_hdl_t *process_get_timouted_hdl(void);

uint8_t process_delete(process_hdl_t *p_hdl);

#endif //ifndef PROCESS_H
