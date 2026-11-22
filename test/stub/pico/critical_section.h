#ifndef GS_STUB_CS_H
#define GS_STUB_CS_H
typedef struct { int x; } critical_section_t;
static inline void critical_section_init(critical_section_t *) {}
static inline void critical_section_enter_blocking(critical_section_t *) {}
static inline void critical_section_exit(critical_section_t *) {}
#endif
