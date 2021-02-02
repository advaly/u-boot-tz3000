#ifndef __EXPORTS_H__
#define __EXPORTS_H__

#ifndef __ASSEMBLY__

/* These are declarations of exported functions available in C code */
unsigned long get_version(void);
int  getc(void);
int  tstc(void);
void putc(const char);
void puts(const char*);
int printf(const char* fmt, ...);
void install_hdlr(int, void (*interrupt_handler_t)(void *), void*);
void free_hdlr(int);
void *malloc(size_t);
void free(void*);
void __udelay(unsigned long);
unsigned long get_timer(unsigned long);
int vprintf(const char *, va_list);
unsigned long simple_strtoul(const char *cp,char **endp,unsigned int base);
int strict_strtoul(const char *cp, unsigned int base, unsigned long *res);
char *getenv (const char *name);
int setenv (const char *varname, const char *varvalue);
long simple_strtol(const char *cp,char **endp,unsigned int base);
int strcmp(const char * cs,const char * ct);
unsigned long ustrtoul(const char *cp, char **endp, unsigned int base);
unsigned long long ustrtoull(const char *cp, char **endp, unsigned int base);
#if defined(CONFIG_CMD_I2C)
int i2c_write (uchar, uint, int , uchar* , int);
int i2c_read (uchar, uint, int , uchar* , int);
#endif

void app_startup(char * const *);

#ifdef CONFIG_STANDALONE_ENABLE_CACHE
void invalidate_icache_all(void);
void flush_cache(unsigned long start, unsigned long size);
void flush_dcache_range(unsigned long start, unsigned long stop);
void invalidate_dcache_all(void);
void invalidate_dcache_range(unsigned long start, unsigned long stop);
#endif

#endif    /* ifndef __ASSEMBLY__ */

enum {
#define EXPORT_FUNC(x) XF_ ## x ,
#include <_exports.h>
#undef EXPORT_FUNC

	XF_MAX
};

#define XF_VERSION	6

#if defined(CONFIG_X86)
extern gd_t *global_data;
#endif

#endif	/* __EXPORTS_H__ */
