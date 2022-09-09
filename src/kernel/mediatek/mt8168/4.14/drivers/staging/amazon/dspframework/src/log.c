/*
 * log.c
 *
 * log management for all kinds of modules on all DSP platforms
 *
 * Copyright 2020-2021 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include "adf/adf_status.h"
#include "adf/adf_common.h"

#define TAG "ADF_LOG"

DEFINE_MUTEX(adfLogLock);

typedef void* (*adf_log_fliter) (char *logLine);

/*
 * adfLog_reCalcHeader()
 * ----------------------------------------
 * re-calculate adf ring buffer header for AP side dumping
 *
 * Input:
 *   adfRingbuf_t *rbuf      - input adf ring buffer
 *   uint32_t flag           - the flag for log module
 *                             higher 16-bit for magic, lower 16-bit for offset
 *   bool chk_overlap     - check the overlap of rp or not
 * Return:
 *   the remain log data size
 */
static int32_t adfLog_reCalcHeader(adfRingbuf_t *rbuf, uint32_t flag, bool chk_overlap)
{
	char check = 0;
	adfRingbuf_t rbufDSP;
	int32_t rptr = (int32_t)(flag & 0xFFFF);
	uint16_t magic = 0;

	/* if the log rbuf is not full-filled, then read from the base */
	if (rbuf->wp < rbuf->limit) {
		rbuf->rp = rbuf->wp + 1;
		adfRingbuf_read(rbuf, &check, 1);
		if (check == 0)
			rbuf->rp = rbuf->base;
	} else
		rbuf->rp = rbuf->base;

	/* read the magic (content) of the pre-set read ptr to see
	 * whether it's overlapped */
	if (ADF_RINGBUF_IN_RANGE(rptr, rbuf->base, rbuf->limit)) {
		if (!chk_overlap) {
			rbuf->rp = rptr;
		} else {
			memcpy(&rbufDSP, rbuf, sizeof(adfRingbuf_t));
			rbuf->rp = ADF_RINGBUF_MOVE_PTR(rptr,
				rbuf->limit - rbuf->base - sizeof(uint16_t),
				rbuf->base, rbuf->limit);
			rbuf->wp = rptr;
			adfRingbuf_read(rbuf, (char *)&magic, sizeof(uint16_t));
			memcpy(rbuf, &rbufDSP, sizeof(adfRingbuf_t));
			if (magic == ((flag >> 16) & 0xFFFF))
				rbuf->rp = rptr;
		}
	}

	return adfRingbuf_getUsedSize(rbuf);
}

/*
 * adfLog_allocAndReadRbuf()
 * ----------------------------------------
 * allocate new buffer and copy the log into the new buffer in seq
 *
 * Input:
 *   adfRingbuf_t *rbuf      - the adf log ring buffer
 *   int32_t size            - the size of the buffer to be allocated and read
 * Return:
 *   NULL or the pointer to the allocated buffer
 */
static char *adfLog_allocAndReadRbuf(adfRingbuf_t *rbuf, int32_t size)
{
	char *buf = NULL;

	/* confirm the size is valid */
	if (size <= 0)
		return NULL;

	/* read all log to tmp buffer */
	buf = (char *)vmalloc(size + 1);
	if (buf == NULL) {
		pr_err("Failed to vmalloc %d for log buffer\n", size + 1);
		return NULL;
	}
	memset(buf, 0, size + 1);
	adfRingbuf_read(rbuf, buf, size);

	return buf;
}

/*
 * adfLog_clearHistory()
 * ----------------------------------------
 * set the RP by force, this only serve the log clear CLI
 *
 * Input:
 *   void *logAddr           - address of log buffer
 *   uint32_t *logFlag       - the pointer to the flag for the log module
 *                             higher 16-bit for magic, lower 16-bit for offset
 * Return:
 *   None
 */
void adfLog_clearHistory(void *logAddr, uint32_t *logFlag)
{
	adfRingbuf_t *rbuf = NULL;

	if (logAddr == NULL) {
		ADF_LOG_E(TAG, "Invalid param logaddr is %p\n", logAddr);
		return ;
	}
	rbuf = ((adfRingbuf_t *) logAddr);

	/* record the last 2 bytes write data as the magic */
	*logFlag = rbuf->wp;
	rbuf->rp = ADF_RINGBUF_MOVE_PTR(rbuf->wp,
		rbuf->limit - rbuf->base - sizeof(uint16_t),
		rbuf->base, rbuf->limit);
	adfRingbuf_read(rbuf, (char *)logFlag + sizeof(uint16_t), sizeof(uint16_t));
}

/*
 * adfLog_dump()
 * ----------------------------------------
 * Dump log to debugfs file
 *
 * Input:
 *   void *logAddr           - address of log buffer
 *   uint32_t *logFlag       - the pointer to the flag for the log module
 *                             higher 16-bit for magic, lower 16-bit for offset
 *   struct seq_file *file   - debugfs file
 * Return:
 *   int
 */
int adfLog_dump(void *logAddr, uint32_t *logFlag, struct seq_file *file)
{
	adfRingbuf_t *rbuf = NULL;
	int32_t remain = 0;
	int32_t ret = 0;
	char *logBuf = NULL;

	if (logAddr == NULL || file == NULL) {
		ADF_LOG_E(TAG, "Invalid param logaddr is %p, file is %p\n",
			logAddr, file);
		return -EINVAL;
	}
	mutex_lock(&adfLogLock);
	rbuf = ((adfRingbuf_t *) logAddr);

	/*
	 * we should always dump all the remaining logs in the ringbuf.
	 * that means, we shouldn't care the RP in the original log rbuf,
	 * the RP should be managed by AP side for this debugfs node dump.
	 */
	remain = adfLog_reCalcHeader(rbuf, *logFlag, true);
	if (remain > 0) {
		/* dump to debugfs file */
		logBuf = adfLog_allocAndReadRbuf(rbuf, remain);
		if (logBuf) {
			seq_printf(file, "%s", logBuf);
			vfree(logBuf);
		} else {
			ret = -ENOMEM;
		}
	}
	mutex_unlock(&adfLogLock);
	return ret;
}

/*
 * adfLog_print()
 * ----------------------------------------
 * print all dsp log to kernel msg
 *
 * Input:
 *   void *logAddr           - address of log buffer
 *   uint32_t* logFlag       - the pointer to the flag for the log module
 *                             higher 16-bit for wp offset,
 *                             lower 16-bit for rp offset
 * Return:
 *   int
 */
int adfLog_print(void *logAddr, uint32_t *logFlag)
{
	adfRingbuf_t *rbuf = NULL;
	int32_t remain = 0;
	int32_t ret = 0;
	char *logBuf = NULL;
	char *logStr = NULL;
	char *logLine = NULL;
	char seps[] = "\r\n";

	if (logAddr == NULL) {
		pr_err("Invalid param logaddr is %p\n", logAddr);
		return -EINVAL;
	}
	mutex_lock(&adfLogLock);
	rbuf = (adfRingbuf_t *) logAddr;

	/*
	 * we should always dump all the remaining logs in the ringbuf.
	 * that means, we shouldn't care the RP in the original log rbuf,
	 * the RP should be managed by AP side for this debugfs node dump.
	 */
	remain = adfLog_reCalcHeader(rbuf, logFlag ? *logFlag : 0, false);
	if (remain > 0) {
		/* split log to line & print to kernel log */
		logBuf = adfLog_allocAndReadRbuf(rbuf, remain);
		if (logBuf) {
			logStr = logBuf;
			logLine = strsep(&logStr, seps);
			while (logLine != NULL) {
				if (*logLine != '\0')
					pr_info("%s\n", logLine);
				logLine = strsep(&logStr, seps);
			}
			vfree(logBuf);
		} else {
			ret = -ENOMEM;
		}
	}
	mutex_unlock(&adfLogLock);
	return ret;
}

/*
 * adfLog_query()
 * ----------------------------------------
 * Query log to get useful information
 *
 * Input:
 *   void *logAddr           - address of log buffer
 *   void *logFliter         - adf log filter func ptr
 * Return:
 *   void* info get from dsp log
 */
void *adfLog_query(void *logAddr, void *logFliter)
{
	adfRingbuf_t *rbuf = NULL;
	int32_t remain = 0;
	char *logBuf = NULL;
	char *logStr = NULL;
	char *logLine = NULL;
	char seps[] = "\r\n";
	adf_log_fliter fliter = (adf_log_fliter)logFliter;
	void *ret = NULL;

	if (logAddr == NULL) {
		pr_err("Invalid param logaddr is %p\n", logAddr);
		return ret;
	}
	mutex_lock(&adfLogLock);
	rbuf = (adfRingbuf_t *)logAddr;

	/*
	 * we should always dump all the remaining logs in the ringbuf.
	 * that means, we shouldn't care the RP in the original log rbuf,
	 * the RP should be managed by AP side for this debugfs node dump.
	 */
	remain = adfLog_reCalcHeader(rbuf, 0, false);
	if (remain > 0) {
		/* read all log to tmp buffer */
		logBuf = adfLog_allocAndReadRbuf(rbuf, remain);
		if (logBuf) {
			logStr = logBuf;
			logLine = strsep(&logStr, seps);
			while (logLine != NULL) {
				if (*logLine != '\0')
					ret = fliter(logLine);
				logLine = strsep(&logStr, seps);
			}
			vfree(logBuf);
		}
	}
	mutex_unlock(&adfLogLock);
	return ret;
}
