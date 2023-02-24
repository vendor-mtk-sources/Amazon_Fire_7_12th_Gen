/*
 * adf_debug.c
 *
 * debugfs include operate of log/cli/state
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

static adf_mem_cpy adf_dbg_mem_read;
static adf_mem_cpy adf_dbg_mem_write;
static int adfDspCoreNo[DSP_CORE_NUM];

/*
 * _adfDebug_logBufCheck()
 * ----------------------------------------
 * Check if log buffer is valid before use it to avoid memory oprate fail cause crash.
 * adf_dbg_mem_read is realized use vendor code, we'd better check it in adf package again.
 *
 * Input:
 *   adfFwHdr_secInfo_t *logInfo - the pointer to the log information stored in dsp header
 * Return:
 *   check result true is pass
 */
static bool _adfDebug_logBufCheck(adfFwHdr_secInfo_t *logInfo)
{
	void *dataPtr = NULL;
	int32_t totalLen = 0;
	bool ret = false;

	if (logInfo && logInfo->data) {
		dataPtr = (char *)(logInfo->data) + sizeof(adfRingbuf_t);
		totalLen = (logInfo->size -
			sizeof(uint32_t) * 2) / ADF_DEBUG_CHAR_LEN -
			sizeof(adfRingbuf_t);
		if (totalLen > 0)
			ret = adfRingbuf_checkValid((adfRingbuf_t *)(logInfo->data),
				dataPtr,
				totalLen);
	}

	if (!ret) {
		pr_err("%s adf log buffer check fail, log ring buffer not valid!", __func__);
	}

	return ret;
}

/*
 * _adfDebug_readLog()
 * ----------------------------------------
 * read log from DSP and dump it to seq_file,
 * this func will be called when user cat debugfs to dump dsp log
 *
 * Input:
 *   struct seq_file *file - the pointer to the target seq_file
 *   void *data            - not used
 * Return:
 *   0 or Error VAL
 */
static int _adfDebug_readLog(struct seq_file *file, void *data)
{
	int ret = 0;
	adfFwHdr_secInfo_t *logInfo;
	adfDspPriv_t *adfDspPriv = NULL;
	uint8_t dspId;

	if (file->private == NULL)
		return -EINVAL;

	dspId = *((int32_t *)file->private) & 0xFF;

	adfDspPriv = adfLoad_getDspPriv(dspId);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			seq_printf(file, "Warning! invalid dspHeader\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			logInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "LOG");
			if (logInfo) {
				memset(logInfo->data, 0, logInfo->size);
				ret = adf_dbg_mem_read((uintptr_t)logInfo->addr,
					(uintptr_t)logInfo->data, logInfo->size);
				if (!ret)
					seq_printf(file, "Warning! Failed to read logData!\n");
				else if (_adfDebug_logBufCheck(logInfo))
					adfLog_dump(logInfo->data, &logInfo->reserved[0], file);
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return 0;
}

/*
 * _adfDebug_writeCli()
 * ----------------------------------------
 * write cli cmd to DSP,
 * this func will be called when user echo debugfs to send cli cmd
 *
 * Input:
 *   struct file *file     - the pointer to the target file, need private_data
 *   const char __user *userbuf - the cli cmd sent from the user space
 *   size_t count          - the length of the cli cmd
 *   loff_t *ppos          - not used
 * Return:
 *   actual write length or Error VAL
 */
static ssize_t _adfDebug_writeCli(struct file *file,
				const char __user *userbuf, size_t count, loff_t *ppos)
{
	int err;
	adfFwHdr_secInfo_t *staInfo;
	adfFwHdr_secInfo_t *cliInfo;
	adfFwHdr_secInfo_t *logInfo;
	adfDspPriv_t *adfDspPriv = NULL;
	char *buf = NULL;
	struct seq_file *seqFile = file->private_data;
	uint8_t dspId;

	if (seqFile->private == NULL)
		return -EINVAL;

	dspId = *((int32_t *)seqFile->private) & 0xFF;

	buf = vmalloc(count + 1);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, count + 1);

	if (copy_from_user(buf, userbuf, count)) {
		vfree(buf);
		return -EFAULT;
	}

	adfDspPriv = adfLoad_getDspPriv(dspId);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader???\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			/* echo clear > $DEBUF_PATH is an exception as
			 * it's for clearing the old log */
			if ((count == ADF_DEBUG_LOG_CLEAR_CMD_LEN) &&
				(0 == memcmp((char *)buf, ADF_DEBUG_LOG_CLEAR_CMD, count))) {
				logInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "LOG");
				if (logInfo) {
					if (adf_dbg_mem_read((uintptr_t)logInfo->addr,
						(uintptr_t)logInfo->data, logInfo->size) > 0 &&
						_adfDebug_logBufCheck(logInfo)) {
						adfLog_clearHistory(logInfo->data,
											&logInfo->reserved[0]);
					}
				}
			} else {
				staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
				cliInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "CLI");

				/* we have to transfer the sta/cli data and
				 * header individually as the lower layer may get the data
				 * before it is fully sent */
				if ((adf_dbg_mem_read((uintptr_t)staInfo->addr,
						(uintptr_t)staInfo->data, staInfo->size) == 0) ||
					(adf_dbg_mem_read((uintptr_t)cliInfo->addr,
						(uintptr_t)cliInfo->data, cliInfo->size) == 0)) {
					pr_err("%s read staData/cliData failed! count %lx\n",
						__func__, (long unsigned int)count);
					mutex_unlock(&adfDspPriv->adfDspLock);
					vfree(buf);
					return count;
				}
				/* always send cli cmd to dsp no matter which state it is */
				adfCli_send(cliInfo->data, (char *)buf, count);
				err = adf_dbg_mem_write((uintptr_t)cliInfo->addr,
					(uintptr_t)cliInfo->data, cliInfo->size);
				if (err) {
					pr_err("%s write cliData failed! count %lx\n",
						__func__, (long unsigned int)count);
				}
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	vfree(buf);
	return count;
}

/*
 * _adfDebug_readState()
 * ----------------------------------------
 * read state from DSP and dump it to seq_file,
 * this func will be called when user cat debugfs to get dsp state
 *
 * Input:
 *   struct seq_file *file - the pointer to the target seq_file
 *   void *data            - not used
 * Return:
 *   0 or Error VAL
 */
static int _adfDebug_readState(struct seq_file *file, void *data)
{
	int ret = 0;
	adfState_t dspState = ADF_STATE_DEF;
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfDspPriv_t *adfDspPriv = NULL;
	uint8_t dspId;

	if (file->private == NULL)
		return -EINVAL;

	dspId = *((int32_t *)file->private) & 0xFF;

	adfDspPriv = adfLoad_getDspPriv(dspId);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			seq_printf(file, "Warning! invalid dspHeader\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			if (staInfo) {
				ret  = adf_dbg_mem_read((uintptr_t)staInfo->addr,
					(uintptr_t)staInfo->data, staInfo->size);
				if (ret) {
					dspState = adfState_get(staInfo->data);
					seq_printf(file, "%02x\n", dspState);
				} else
					seq_printf(file, "Warning! "
						"failed to read state from dsp!\n");
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return 0;
}

/*
 * _adfDebug_writeState()
 * ----------------------------------------
 * write dsp state to DSP,
 * this func will be called when user echo debugfs to set dsp state
 *
 * Input:
 *   struct file *file     - the pointer to the target file, need private_data
 *   const char __user *userbuf - the cli cmd sent from the user space
 *   size_t count          - the length of the cli cmd
 *   loff_t *ppos          - not used
 * Return:
 *   actual write length or Error VAL
 */
static ssize_t _adfDebug_writeState(struct file *file,
				const char __user *userbuf, size_t count, loff_t *ppos)
{
	int ret = 0;
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfDspPriv_t *adfDspPriv = NULL;
	char *buf = NULL;
	struct seq_file *seqFile = file->private_data;
	uint8_t sta;
	uint8_t dspId;

	if (seqFile->private == NULL)
		return -EINVAL;

	dspId = *((int32_t *)seqFile->private) & 0xFF;

	buf = vmalloc(count + 1);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, count + 1);

	if (copy_from_user(buf, userbuf, count)) {
		vfree(buf);
		return -EFAULT;
	}

	adfDspPriv = adfLoad_getDspPriv(dspId);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader???\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			if (staInfo == NULL) {
				mutex_unlock(&adfDspPriv->adfDspLock);
				vfree(buf);
				return count;
			}
			if (kstrtou8(buf, 16, &sta) == 0) {
				ret = adf_dbg_mem_read((uintptr_t)staInfo->addr,
					(uintptr_t)staInfo->data, staInfo->size);
				if (!ret) {
					mutex_unlock(&adfDspPriv->adfDspLock);
					vfree(buf);
					return count;
				}
				adfState_set(staInfo->data, sta);
				ret = adf_dbg_mem_write((uintptr_t)staInfo->addr,
					(uintptr_t)staInfo->data, staInfo->size);
				if (ret)
					pr_err("Warning! state write to dsp fail!\n");
				else
					pr_info("Set dsp state to 0x%02x\n", sta);
			} else {
				pr_err("Warning! invalid state input!\n");
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	vfree(buf);
	return count;
}

/*
 * _adfDebug_open()
 * ----------------------------------------
 * open API for log/cli debug node, link the readLog to seq_file hdls
 *
 * Input:
 *   struct inode *inode   - the pointer to the operation node
 *   struct file *file     - the file handler, seq_file is file->private_data
 * Return:
 *   0 or Error VAL
 */
static int _adfDebug_open(struct inode *inode, struct file *file)
{
	return single_open(file, _adfDebug_readLog, inode->i_private);
}

/*
 * _adfDebug_openState()
 * ----------------------------------------
 * open API for state debug node, link the readState to seq_file hdls
 *
 * Input:
 *   struct inode *inode   - the pointer to the operation node
 *   struct file *file     - the file handler, seq_file is file->private_data
 * Return:
 *   0 or Error VAL
 */
static int _adfDebug_openState(struct inode *inode, struct file *file)
{
	return single_open(file, _adfDebug_readState, inode->i_private);
}

/* The FS operation handlers for log/cli and state debug nodes */
static struct file_operations adfDebugFops = {
	.owner = THIS_MODULE,
	.open = _adfDebug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = _adfDebug_writeCli,
};

static struct file_operations adfDebugStateFops = {
	.owner = THIS_MODULE,
	.open = _adfDebug_openState,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = _adfDebug_writeState,
};

/*
 * adfDebug_initFs()
 * ----------------------------------------
 * The entry to init the FS for the cli/log and state debug nodes
 * The read/write APIs may be different on each platform, considered as HAL
 *
 * Input:
 *   void *debugReadFunc   - the function pointer to the debug read API
 *   void *debugWriteFunc  - the function pointer to the debug write API
 * Return:
 *   0 or Error VAL
 */
int adfDebug_initFs(void *debugReadFunc, void *debugWriteFunc)
{
	struct dentry *adfDebugDir = NULL;
	struct dentry *adfDebugFile = NULL;
	char debugFileName[12] = {0};
	int i = 0;

	if (debugReadFunc == NULL || debugWriteFunc == NULL) {
		pr_err("Invalid param read func %p, write func %p\n",
			debugReadFunc, debugWriteFunc);
		return -EINVAL;
	}
	adf_dbg_mem_read = debugReadFunc;
	adf_dbg_mem_write = debugWriteFunc;

	if (DSP_CORE_NUM >= 10 || DSP_CORE_NUM <= 0) {
		pr_err("Invalid param dsp core num %d\n", DSP_CORE_NUM);
		return -EINVAL;
	}

	adfDebugDir = debugfs_create_dir("adf_dbg_fs", NULL);
	if (!adfDebugDir)
		return -ENOMEM;

	for (i = 0; i < DSP_CORE_NUM; i++) {
		adfDspCoreNo[i] = i;
		snprintf(debugFileName, sizeof(debugFileName), "adf_debug_%d", i);
		adfDebugFile = debugfs_create_file(debugFileName,
						0644,
						adfDebugDir,
						&adfDspCoreNo[i],
						&adfDebugFops);
		if (!adfDebugFile)
				goto fail;

		snprintf(debugFileName, sizeof(debugFileName), "adf_state_%d", i);
		adfDebugFile = debugfs_create_file(debugFileName,
							0644,
							adfDebugDir,
							&adfDspCoreNo[i],
							&adfDebugStateFops);
		if (!adfDebugFile)
				goto fail;
	}
	return 0;

fail:
	debugfs_remove_recursive(adfDebugDir);
	return -ENOMEM;
}

/*
 * adfDebug_query()
 * ----------------------------------------
 * Send the debug cli cmd and query the feedback logs
 * Note that we'll just dump the filtered log
 *
 * Input:
 *   uint8_t dspId         - the ID of the target DSP core
 *   char **cliCmds        - the pointer to the cli command aray
 *   uint8_t cliCmdNum     - the count of the array
 *   void *logFliter       - the filter to the feedback log
 * Return:
 *   NULL or void* info get from dsp log
 */
void *adfDebug_query(uint8_t dspId, char **cliCmds, uint8_t cliCmdNum,
				void *logFliter)
{
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfFwHdr_secInfo_t *cliInfo = NULL;
	adfFwHdr_secInfo_t *logInfo = NULL;
	adfState_t dspState = ADF_STATE_DEF;
	adfDspPriv_t *adfDspPriv = NULL;
	void *ret = NULL;
	int i = 0;

	adfDspPriv = adfLoad_getDspPriv(dspId);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader %s, %d\n", __FILE__, __LINE__);
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			/* send cli command to trigger */
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			cliInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "CLI");
			logInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "LOG");
			if (staInfo == NULL || cliInfo == NULL || logInfo == NULL) {
				mutex_unlock(&adfDspPriv->adfDspLock);
				return ret;
			}

			memset(staInfo->data, 0, staInfo->size);
			if (adf_dbg_mem_read((uintptr_t)staInfo->addr,
				(uintptr_t)staInfo->data, staInfo->size) != 0)
				dspState = adfState_get(staInfo->data);

			if (dspState < ADF_STATE_RUN || dspState >= ADF_STATE_ERR) {
				pr_err("Warning! invalid ADF state 0x%02x\n", dspState);
				mutex_unlock(&adfDspPriv->adfDspLock);
				return ret;
			}

			for (i = 0; i < cliCmdNum; i++) {
				memset(cliInfo->data, 0, cliInfo->size);
				if (adf_dbg_mem_read((uintptr_t)cliInfo->addr,
					(uintptr_t)cliInfo->data, cliInfo->size) > 0) {
						adfCli_send(cliInfo->data, *(cliCmds + i),
							strlen(*(cliCmds + i)));
						adf_dbg_mem_write((uintptr_t)cliInfo->addr,
							(uintptr_t)cliInfo->data, cliInfo->size);
						/* sleep instead of check ack to aviod dsp crash or
						* not run cause kernel hang here */
						msleep (100);
				} else
					pr_err("Warning! can't send adf cli command\n");
			}

			memset(logInfo->data, 0, logInfo->size);
			if (adf_dbg_mem_read((uintptr_t)logInfo->addr,
				(uintptr_t)logInfo->data, logInfo->size) > 0 &&
				_adfDebug_logBufCheck(logInfo))
				ret = adfLog_query(logInfo->data, logFliter);
			else
				pr_err("Warning! can't read adf log\n");

		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return ret;
}

/*
 * adfDebug_queryState()
 * ----------------------------------------
 * get the DSP state of the specific DSP core
 *
 * Input:
 *   uint8_t dspId         - the ID of the target DSP core
 * Return:
 *   the current dsp state
 */
int adfDebug_queryState(uint8_t dspId)
{
	int ret = 0;
	adfState_t dspState = ADF_STATE_DEF;
	adfFwHdr_secInfo_t *staInfo = NULL;
	adfDspPriv_t *adfDspPriv = NULL;

	adfDspPriv = adfLoad_getDspPriv(dspId);
	if (adfDspPriv) {
		mutex_lock(&adfDspPriv->adfDspLock);
		if (adfDspPriv->dspHeader == NULL)
			pr_err("Warning! invalid dspHeader\n");
		else if (adfDspPriv->dspHeader->magic == ADF_FW_MAGIC) {
			staInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "STA");
			if (staInfo) {
				ret = adf_dbg_mem_read((uintptr_t)staInfo->addr,
					(uintptr_t)staInfo->data, staInfo->size);
				if (ret)
					dspState = adfState_get(staInfo->data);
				else
					pr_err("Warning! failed to read state from dsp!\n");
			}
		}
		mutex_unlock(&adfDspPriv->adfDspLock);
	}
	return dspState;
}

/*
 * adfDebug_printLog()
 * ----------------------------------------
 * read log from DSP and print it to kernel log (dmesg),
 * this func will be called periodically in driver, or when DSP is crashed
 *
 * Input:
 *   uint8_t dspId         - the index of the DSP core
 *   uint32_t mode         - the log print mode, 0 for RECENT and 1 for ALL
 * Return:
 *   None
 */
void adfDebug_printLog(uint8_t dspId, uint32_t mode)
{
	adfFwHdr_secInfo_t *logInfo;
	adfDspPriv_t *adfDspPriv = adfLoad_getDspPriv(dspId);
	int32_t wptr = 0;

	/* confirm whether the core index is valid, and we can get the priv */
	if (adfDspPriv == NULL)
		return ;
	mutex_lock(&adfDspPriv->adfDspLock);
	logInfo = adfLoad_GetSecInfo(adfDspPriv->dspHeader, "LOG");

	/* mode = 0 means periodical print dsp log to kernel log,
	 *     only the new log should be printed
	 * mode = 1 means print dsp log to kernel log when dsp is crashed,
	 *     all the valid log in the buffer should be printed */
	if (mode == ADF_LOG_DUMP_RECENT) {
		/*
		 * we will read the write pointer first,
		 * if it is not changed, then that means no new log.
		 * Note that, we understand the best way here should check
		 * both RP and WP and magic,
		 * but it's not necessary to do it because the log ring buf
		 * won't be full within several seconds.
		 *
		 * The definition of the uint32_t flag is that,
		 * higher 16 bits for wp offset, lower 16 bits for rp offset
		 */
		wptr = ((adfRingbuf_t *)(logInfo->data))->wp;
		if (adf_dbg_mem_read((uintptr_t)logInfo->addr,
			(uintptr_t)logInfo->data, sizeof(adfRingbuf_t)) > 0) {
			if (wptr == logInfo->reserved[1]) {
				mutex_unlock(&adfDspPriv->adfDspLock);
				return ;
			}
		}
		if (adf_dbg_mem_read((uintptr_t)logInfo->addr,
			(uintptr_t)logInfo->data, logInfo->size) > 0 &&
			_adfDebug_logBufCheck(logInfo))
			adfLog_print(logInfo->data, &logInfo->reserved[1]);
		logInfo->reserved[1] = wptr;
	} else if (mode == ADF_LOG_DUMP_ALL) {
		if (adf_dbg_mem_read((uintptr_t)logInfo->addr,
			(uintptr_t)logInfo->data, logInfo->size) <= 0)
			pr_warn("Warning! Failed to read logData!\n");
		else if (_adfDebug_logBufCheck(logInfo))
			adfLog_print(logInfo->data, NULL);
	}
	mutex_unlock(&adfDspPriv->adfDspLock);
}
