// SPDX-License-Identifier: GPL-2.0-only
/*
 * Register read and write tracepoints
 *
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "trace_readwrite: " fmt

#include <linux/ftrace.h>
#include <linux/module.h>
#include <linux/io.h>

#define CREATE_TRACE_POINTS
#include <trace/events/rwmmio.h>

#ifdef CONFIG_TRACE_MMIO_ACCESS
volatile void __iomem *log_mmio_blocklist[8];

struct {
	volatile void __iomem *start;
	volatile void __iomem *end;
	const char *tag;
} log_mmio_ranges[8];

void log_mmio_register_block(volatile void __iomem *addr)
{
	int i;

	for (i=0; i < ARRAY_SIZE(log_mmio_blocklist); i++) {
		if (log_mmio_blocklist[i])
			continue;

		log_mmio_blocklist[i] = addr;
		return;
	}

	pr_err("Failed to block MMIO accesses to %p\n", addr);
}
EXPORT_SYMBOL_GPL(log_mmio_register_block);

void log_mmio_register_range(volatile void __iomem *addr, unsigned long len, const char *tag)
{
	int i;

	for (i=0; i < ARRAY_SIZE(log_mmio_ranges); i++) {
		if (log_mmio_ranges[i].start)
			continue;

		log_mmio_ranges[i].start = addr;
		log_mmio_ranges[i].end = addr + len;
		log_mmio_ranges[i].tag = tag;

		pr_info("Registered MMIO range for %s\n", tag);
		return;
	}

	pr_err("Failed to register MMIO range for %s\n", tag);
}
EXPORT_SYMBOL_GPL(log_mmio_register_range);

static int lookup_mmio_register(const volatile void __iomem *addr)
{
	int i;

	for (i=0; i < ARRAY_SIZE(log_mmio_blocklist); i++)
		if (addr == log_mmio_blocklist[i])
			return -1;

	for (i=0; i < ARRAY_SIZE(log_mmio_ranges); i++) {
		if (log_mmio_ranges[i].start < addr &&
		    log_mmio_ranges[i].end > addr)
			return i;
	}

	return -1;
}

void log_write_mmio(u64 val, u8 width, volatile void __iomem *addr,
		    unsigned long caller_addr, unsigned long caller_addr0)
{
	int i = lookup_mmio_register(addr);
	if (i >= 0)
		trace_rwmmio_write(caller_addr, caller_addr0, val, width, addr,
				   log_mmio_ranges[i].tag,
				   addr - log_mmio_ranges[i].start);
}
EXPORT_SYMBOL_GPL(log_write_mmio);
EXPORT_TRACEPOINT_SYMBOL_GPL(rwmmio_write);

void log_post_write_mmio(u64 val, u8 width, volatile void __iomem *addr,
			 unsigned long caller_addr, unsigned long caller_addr0)
{
	int i = lookup_mmio_register(addr);
	if (i >= 0)
		trace_rwmmio_post_write(caller_addr, caller_addr0, val, width,
					addr, log_mmio_ranges[i].tag,
					addr - log_mmio_ranges[i].start);
}
EXPORT_SYMBOL_GPL(log_post_write_mmio);
EXPORT_TRACEPOINT_SYMBOL_GPL(rwmmio_post_write);

void log_read_mmio(u8 width, const volatile void __iomem *addr,
		   unsigned long caller_addr, unsigned long caller_addr0)
{
	int i = lookup_mmio_register(addr);
	if (i >= 0)
		trace_rwmmio_read(caller_addr, caller_addr0, width, addr,
				  log_mmio_ranges[i].tag,
				  addr - log_mmio_ranges[i].start);
}
EXPORT_SYMBOL_GPL(log_read_mmio);
EXPORT_TRACEPOINT_SYMBOL_GPL(rwmmio_read);

void log_post_read_mmio(u64 val, u8 width, const volatile void __iomem *addr,
			unsigned long caller_addr, unsigned long caller_addr0)
{
	int i = lookup_mmio_register(addr);
	if (i >= 0)
		trace_rwmmio_post_read(caller_addr, caller_addr0, val, width,
				       addr, log_mmio_ranges[i].tag,
				       addr - log_mmio_ranges[i].start);
}
EXPORT_SYMBOL_GPL(log_post_read_mmio);
EXPORT_TRACEPOINT_SYMBOL_GPL(rwmmio_post_read);
#endif /* CONFIG_TRACE_MMIO_ACCESS */
