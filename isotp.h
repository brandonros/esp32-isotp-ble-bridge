#ifndef ISOTP_H
#define ISOTP_H

#define ISOTP_BUFSIZE 4096

static uint8_t *isotp_payload_buffer;

void isotp_poll_task_callback();
void isotp_receive_task_callback();
void isotp_setup();

#endif
