#ifndef CASCADE_LEFT_H
#define CASCADE_LEFT_H
#include "cascade_link.h"
extern volatile CascadeDiagnostics g_cascade_left_diag;
void cascade_left_init(void);
void cascade_left_poll(void);
/* Safe from main or interrupts; independent of existing control inputs. */
void cascade_left_get_snapshot(CascadeSnapshot *out);
#endif

