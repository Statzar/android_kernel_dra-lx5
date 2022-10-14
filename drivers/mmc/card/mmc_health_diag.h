#ifndef MMC_HEALTH_DIAG_H
#define MMC_HEALTH_DIAG_H

#define MAX_ERR_TIMES 10
#define MAX_ERR_TYPE 5
#define PRT_TIME_PERIOD 10000000000  //10s
#define SD_IO_BUSY 95
#define MAX_REPORT_TIMES 10
#define LOW_SPEED_WARTING_VALUE 1024
#define MAX_WRITE_SPEED_ABNOR_TIMES 4

unsigned int mmc_calculate_ioworkload_and_rwspeed(unsigned long long time,struct request *rqc,struct gendisk *disk);
int mmc_calculate_rw_size(unsigned long long time,unsigned int rq_byte,struct request *rqc);
extern unsigned int mmc_get_sd_speed(void);
#endif
