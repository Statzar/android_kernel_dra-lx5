/*
 *  linux/drivers/mmc/card/mmc_health_check.c
 *
 *  Copyright (c) 2013 Huawei Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.

 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/string_helpers.h>
#include <linux/delay.h>
#include <linux/capability.h>
#include <linux/compat.h>
#include <linux/syscalls.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mmc/core.h>
#include <linux/pm_runtime.h>
#include <trace/events/mmc.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include "mmc_health_diag.h"

static unsigned long long mmccid_tag_t1 = 0;//the time stamp the last rw
static unsigned int mmcqd_rq_count = 0;//the rw cmd times in one stat cycle
static unsigned long long mmcqd_t_usage_wr=0,mmcqd_t_usage_rd=0;//the time which spent in function mmc_blk_issue_rw_rq in one stat cycle
static unsigned int mmcqd_rq_size_wr=0,mmcqd_rq_size_rd=0;//the size which read and write totally in one stat cycle
static unsigned int wr_speed_abnor_times = 0;// the times io speed abnormally


/*===========================================================================
 * FUNCTION: mmc_calculate_ioworkload_and_rwspeed
   PARAMETER:
 *    @time: the current time
 *    @rqc: the request which will be sent to host .
 *    @disk:the disk which has been monitor
 *
 * Description: calculate the ioworkload and rwspeed,to determine whether the bad
                card message to be reported
 *
 * Returns: the size which this request will send

===========================================================================*/
unsigned int mmc_calculate_ioworkload_and_rwspeed(unsigned long long time, struct request *rqc, struct gendisk *disk)
{
    unsigned long long t_period=0,t_usage = 0, t_tmp = 0;
    unsigned int rq_byte=0;
    unsigned int t_percent=0;
    unsigned int t_percent_wr=0;
    unsigned int perf_meter=0;
    pid_t mmcqd_pid=0;

    mmcqd_pid = task_pid_nr(current);
    if(mmccid_tag_t1==0) {
        mmccid_tag_t1 = time;
    }

    //get the period between this rw and the last rw
    t_period = time - mmccid_tag_t1;

    if(rqc) {
        //get the rw size this rw
        rq_byte = blk_rq_bytes(rqc);
    }

    //if period > 10s ,so stat
    if(t_period >= (unsigned long long )PRT_TIME_PERIOD) {
        t_usage = mmcqd_t_usage_wr + mmcqd_t_usage_rd;// total real rw time
       printk("mmcqd/1 rw time:%lld ns,write time:%lld ns,read time:%lld ns\n",t_usage,mmcqd_t_usage_wr,mmcqd_t_usage_rd);
        /* worload < 0.01*/
        if(t_period > t_usage*100) {        // io workload < 1%
            printk("mmcqd/1 workload < 1%%, duty %lld, period %lld, req_cnt=%d\n",t_usage, t_period, mmcqd_rq_count);
        } else {
            do_div(t_period, 100);// divided by 100,just to get %
            t_tmp = t_usage;
            do_div(t_tmp, t_period);
            t_percent = (unsigned int)t_tmp;
            t_tmp = mmcqd_t_usage_wr;
            do_div(t_tmp, t_period);
            t_percent_wr = (unsigned int)t_tmp;
            printk("mmcqd/1 rw_workload = %d%%, w_workload = %d%%, duty %lld, period %lld00, req_cnt=%d\n",t_percent, t_percent_wr, t_usage, t_period,mmcqd_rq_count);
        }

        /* get write speed*/
        if(mmcqd_t_usage_wr)
        {
            //change ns to ms!
            do_div(mmcqd_t_usage_wr, 1000000);
            if(mmcqd_t_usage_wr) { //if >=1ms
                perf_meter = (mmcqd_rq_size_wr)/((unsigned int)mmcqd_t_usage_wr);
                if((t_percent_wr >= SD_IO_BUSY) &&(perf_meter < LOW_SPEED_WARTING_VALUE)) {
                    wr_speed_abnor_times++;
                    if(wr_speed_abnor_times == MAX_WRITE_SPEED_ABNOR_TIMES) {
                        wr_speed_abnor_times = 0;
                        printk(KERN_ERR "mmcqd/1:sd_spec:%d,the card wr_speed is lower than spec\n",mmc_get_sd_speed());
                    }
                } else {
                     wr_speed_abnor_times = 0;
                }
            } else {
              wr_speed_abnor_times = 0;
            }
            printk("mmcqd/1 Write Throughput=%d KB/s, size: %d bytes, time:%lld ms, pid: %d, name:%s\n",perf_meter,mmcqd_rq_size_wr,mmcqd_t_usage_wr, task_pid_nr(current), current->comm);
        } else {
           wr_speed_abnor_times = 0;
        }

        /* get read speed*/
        if(mmcqd_t_usage_rd) {
            do_div(mmcqd_t_usage_rd, 1000000);
            if(mmcqd_t_usage_rd) {
                perf_meter = (mmcqd_rq_size_rd)/((unsigned int)mmcqd_t_usage_rd);
                printk("mmcqd/1 Read Throughput=%d kB/s, size: %d bytes, time:%lld ms, pid: %d, name: %s\n",perf_meter,mmcqd_rq_size_rd,mmcqd_t_usage_rd, task_pid_nr(current), current->comm);
            }
        }

        mmccid_tag_t1 = time;
        mmcqd_t_usage_wr = 0;
        mmcqd_t_usage_rd = 0;
        mmcqd_rq_count = 0;
        mmcqd_rq_size_wr = 0;
        mmcqd_rq_size_rd = 0;
    }
    return rq_byte;
}

/*===========================================================================
 * FUNCTION: mmc_calculate_rw_size
   PARAMETER:
 *    @time: the time when the function mmc_blk_issue_rw_rq was entered
 *    @rq_byte: the size which this request will send .
 *    @rqc:the request which has been sent to host
 *
 * Description: at the end of mmc_blk_issue_rw_rq,to record the total rw size
                and rw time for
 *
 * Returns: 0-----SUCCESS

===========================================================================*/
int mmc_calculate_rw_size(unsigned long long time, unsigned int rq_byte, struct request *rqc)
{
   unsigned long long time2 = 0;

    //get current time from the phone boot ,ns!
    time2 = sched_clock();
    if(rqc) {
        if(rq_data_dir(rqc) == WRITE) {
            mmcqd_t_usage_wr += time2-time;
            mmcqd_rq_size_wr += rq_byte;
        } else {
            mmcqd_t_usage_rd += time2-time;
            mmcqd_rq_size_rd += rq_byte;
        }

        mmcqd_rq_count ++;
    }
    return 0;
}
