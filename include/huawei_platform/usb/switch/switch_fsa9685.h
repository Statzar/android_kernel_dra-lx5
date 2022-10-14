
#include <linux/hisi/usb/hisi_usb.h>
#include <huawei_platform/usb/switch/switch_fsa9685_common.h>

#define ADAPTOR_BC12_TYPE_MAX_CHECK_TIME 100
#define WAIT_FOR_BC12_DELAY 5
#define ACCP_NOT_PREPARE_OK -1
#define ACCP_PREPARE_OK 0
#define BOOST_5V_CLOSE_FAIL -1
#define SET_DCDTOUT_FAIL -1
#define SET_DCDTOUT_SUCC 0


int fcp_read_switch_status (void);
int fcp_read_adapter_status(void);
void switch_dump_register(void);
int is_fcp_charger_type(void);
