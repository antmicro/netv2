#ifndef __PROCESSOR_H
#define __PROCESSOR_H

#define PROCESSOR_MODE_COUNT 15
#define PROCESSOR_MODE_DESCLEN 64

void hdmi_in_setup(int mode);
void hdmi_out_setup(int mode);
void hdmi_service(void);

#endif /* __PROCESSOR_H */
