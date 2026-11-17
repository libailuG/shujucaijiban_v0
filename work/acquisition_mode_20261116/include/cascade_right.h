#ifndef CASCADE_RIGHT_H
#define CASCADE_RIGHT_H
#include "cascade_link.h"
extern volatile CascadeDiagnostics g_cascade_right_diag;
void cascade_right_init(void);
void cascade_right_poll(void);
/* Safe from main or interrupts; independent of existing control inputs. */
void cascade_right_get_snapshot(CascadeSnapshot *out);
#endif

