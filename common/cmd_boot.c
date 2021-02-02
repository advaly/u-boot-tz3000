/*
 * (C) Copyright 2000-2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Misc boot support
 */
#include <common.h>
#include <command.h>
#include <net.h>

#ifdef CONFIG_CMD_GO

/* Allow ports to override the default behavior */
__attribute__((weak))
unsigned long do_go_exec(ulong (*entry)(int, char * const []), int argc,
				 char * const argv[])
{
	return entry (argc, argv);
}

static int do_go(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong	addr, rc;
	int     rcode = 0;

	if (argc < 2)
		return CMD_RET_USAGE;

	addr = simple_strtoul(argv[1], NULL, 16);

	printf ("## Starting application at 0x%08lX ...\n", addr);

#ifdef CONFIG_STANDALONE_ENABLE_CACHE
	flush_dcache_all();
	invalidate_dcache_all();
	invalidate_icache_all();
#elif defined(CONFIG_STANDALONE_DISABLE_CACHE)
	cleanup_before_linux();
#endif
#ifdef CONFIG_DISPLAY_BOOTTIME
	if (getenv("boottime") && getenv("bootcmd"))
		boottime();
#endif
	/*
	 * pass address parameter as argv[0] (aka command name),
	 * and all remaining args
	 */
	rc = do_go_exec ((void *)addr, argc - 1, argv + 1);
	if (rc != 0) rcode = 1;

#ifdef CONFIG_STANDALONE_DISABLE_CACHE
	setup_after_linux();
#endif
	printf ("## Application terminated, rc = 0x%lX\n", rc);
	return rcode;
}

/* -------------------------------------------------------------------- */

U_BOOT_CMD(
	go, CONFIG_SYS_MAXARGS, 1,	do_go,
	"start application at address 'addr'",
	"addr [arg ...]\n    - start application at address 'addr'\n"
	"      passing 'arg' as arguments"
);

#endif

U_BOOT_CMD(
	reset, 1, 0,	do_reset,
	"Perform RESET of the CPU",
	""
);
