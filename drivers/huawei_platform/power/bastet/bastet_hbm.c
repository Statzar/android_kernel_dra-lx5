/* bastet_hbm.c
 *
 * Bastet heartbeat reply content management.
 *
 * Copyright (C) 2015 Huawei Device Co.,Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/file.h>
#include <linux/net.h>
#include <net/sock.h>

#include <huawei_platform/power/bastet/bastet_utils.h>
#include <huawei_platform/power/bastet/bastet.h>


int bastet_send_msg(struct bst_sock_id *guide, uint8_t *data, uint32_t len)
{
	int ret = 0;
	struct sock *sk;
	struct msghdr msg;
	struct kvec iov[1];

	/* parameters check */
	if (guide == NULL || data == NULL || len == 0) {
		BASTET_LOGE("invalid parameters");
		return -EFAULT;
	}
	/* find sock according to fd & pid */
	sk = get_sock_by_fd_pid(guide->fd, guide->pid);
	if (NULL == sk) {
		BASTET_LOGE("can not find sock by fd: %d pid: %d",
					guide->fd, guide->pid);
		return -ENOENT;
	}
	memset(&msg, 0, sizeof(msg));
	memset(&iov, 0, sizeof(iov));
	msg.msg_flags  |= MSG_DONTWAIT;
	iov[0].iov_base = data;
	iov[0].iov_len  = len;

	/* send data from kernel directly */
	ret = kernel_sendmsg(sk->sk_socket, &msg, iov, 1, len);
	if (ret != len) {
		BASTET_LOGE("kernel_sendmsg failed, ret=%d", ret);
		sock_put(sk);
		return -EFAULT;
	}
	sock_put(sk);

	return 0;
}
int bastet_set_hb_reply(struct bst_sock_id *guide, uint8_t *data, uint32_t len)
{
	struct sock *sk;
	struct bastet_sock *bsk;
	int err = 0;

	/* parameters check */
	if (guide == NULL || data == NULL || len == 0) {
		BASTET_LOGE("Invalid input parameters");
		err = -EFAULT;
		goto out;
	}
	/* find sock according to fd & pid */
	sk = get_sock_by_fd_pid(guide->fd, guide->pid);
	if (NULL == sk) {
		BASTET_LOGE("can not find sock by fd: %d pid: %d",
					guide->fd, guide->pid);
		err = -ENOENT;
		goto out;
	}
	bsk = sk->bastet;
	if (!bsk) {
		BASTET_LOGE("no bastet in sock");
		err = -ENOENT;
		goto out_put;
	}
	if (bsk->hbm.reply_content != NULL) {
		if (memcmp(bsk->hbm.reply_content, data,
			bsk->hbm.reply_len) == 0) {
			BASTET_LOGI(
				"heartbeat reply content is exist,sk: %p",
				sk);
			goto out_put;
		} else {
			/* heartbeat reply content is different,*/
			/* clear original one */
			BASTET_LOGI(
				"update heartbeat reply content, sk: %p",
				sk);
			kfree(bsk->hbm.reply_content);
			bsk->hbm.reply_content = NULL;
			bsk->hbm.reply_len = 0;
		}
	}

	bsk->hbm.reply_len = len;
	bsk->hbm.reply_content = data;
	BASTET_LOGI("add heartbeat reply content, sk: %p", sk);
out_put:
	sock_put(sk);
out:
	return err;
}


void bastet_clear_hb_reply(struct bastet_sock *bsk)
{
	if (bsk && bsk->hbm.reply_content) {
		BASTET_LOGI("kfree heartbeat reply content");
		kfree(bsk->hbm.reply_content);
		bsk->hbm.reply_content = NULL;
		bsk->hbm.reply_len = 0;
	}
}


static void bastet_notify_reply_recv(struct sock *sk, bst_ind_type type)
{
	struct bst_sock_comm_prop comm_prop;
	int ret = 0;

	ret = bastet_get_comm_prop(sk, &comm_prop);
	if (ret != 0) {
		BASTET_LOGE("failed to get comm prop, ret=%d", ret);
		return;
	}
	post_indicate_packet(type, &comm_prop,
		sizeof(struct bst_sock_comm_prop));
}


static int bastet_drop_hb_reply(struct sock *sk, struct msghdr *msg, int len)
{
	struct bastet_sock *bsk = sk->bastet;
	int offset = bsk->hbm.reply_offset;
	int ret = len;
	bool notMatched = 1;
	unsigned int minLen = 0;

	if (len < 0) {
		return -EFAULT;
	}
	if ( (NULL == msg) || (NULL == bsk) ) {
		return -EFAULT;
	}
	if (!access_ok(VERIFY_READ, msg->msg_iter.iov->iov_base, len)) {
		return -EFAULT;
	}
	minLen = len > bsk->hbm.reply_len ? bsk->hbm.reply_len : len;

	if (bsk->hbm.reply_content) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 10)
		if(msg->msg_iter.iov->iov_base != NULL) {
			notMatched = memcmp(bsk->hbm.reply_content + offset,
			    msg->msg_iter.iov->iov_base, minLen);
		}
#else
		notMatched = memcmp(bsk->hbm.reply_content + offset,
			msg->msg_iov->iov_base - len, minLen);
#endif
		if ( !notMatched ) {
			bsk->hbm.reply_offset += len;
			/* check whether heartbeat reply */
			/* matching has been completed */
			if (bsk->hbm.reply_offset == bsk->hbm.reply_len) {
				bsk->hbm.reply_offset = 0;
				atomic_dec(&bsk->hbm.reply_matched_cnt);
			}
			/*
			 * return fake interrupt error to
			 * make userspace ignores received heartbeat reply
			 */
			ret = -EINTR;
		}
	}

	return ret;
}


inline int bastet_check_hb_reply(struct sock *sk, struct msghdr *msg, int err)
{
	/* check wifi proxy socket when heartbeat reply matched */
	if (is_wifi_proxy(sk)
		&& (atomic_read(&sk->bastet->hbm.reply_matched_cnt) > 0)) {
		return bastet_drop_hb_reply(sk, msg, err);
	}

	return err;
}
static void bastet_match_hb_reply(struct sock *sk, uint8_t *data, int len)
{
	struct bastet_sock *bsk = sk->bastet;

	if (!data || len <= 0) {
		BASTET_LOGE("Invalid parameters");
		return;
	}
	if (bsk->hbm.reply_content) {
		/* compare heartbeat reply content */
		if (memcmp(bsk->hbm.reply_content, data, len) == 0) {
			BASTET_LOGI("len: %d", len);
			/* notify bastetd heartbeat reply packet has received */
			bastet_notify_reply_recv(sk, BST_IND_HB_REPLY_RECV);
			atomic_dec(&bsk->hbm.reply_filter_cnt);
			atomic_inc(&sk->bastet->hbm.reply_matched_cnt);
		} else if (atomic_read(&sk->bastet->hbm.frozen) > 0) {
			/* it is not heartbeat reply, */
			/* notify bastetd to thaw the process */
			atomic_set(&sk->bastet->hbm.frozen, 0);
			post_indicate_packet(BST_IND_TRIGGER_THAW,
				&sk->bastet->pid, sizeof(pid_t));
		}
	}
}


inline void bastet_mark_hb_reply(struct sock *sk, struct sk_buff *skb, int len)
{
	/* only check wifi proxy socket */
	if (is_wifi_proxy(sk)) {
		/* only has sent heartbeat packet *
		/* needs match heartbeat reply packet */
		if (atomic_read(&sk->bastet->hbm.reply_filter_cnt) > 0) {
			bastet_match_hb_reply(sk,
				skb->data + len, skb->len - len);
		} else if (atomic_read(&sk->bastet->hbm.frozen) > 0) {
			/* it is not heartbeat reply, */
			/* notify bastetd to thaw the process */
			atomic_set(&sk->bastet->hbm.frozen, 0);
			post_indicate_packet(BST_IND_TRIGGER_THAW,
				&sk->bastet->pid, sizeof(pid_t));
		}
	}
}


int bastet_filter_hb_reply(struct bst_sock_id *guide)
{
	struct sock *sk;
	int err = 0;

	if (guide == NULL) {
		BASTET_LOGE("Invalid input parameters");
		return -EFAULT;
	}
	/* find sock according to fd & pid */
	sk = get_sock_by_fd_pid(guide->fd, guide->pid);
	if (NULL == sk) {
		BASTET_LOGE("can not find sock by fd: %d pid: %d",
			guide->fd, guide->pid);
		return -ENOENT;
	}
	if (is_wifi_proxy(sk)) {
		BASTET_LOGI("filter heartbeat reply, sk: %p", sk);
		atomic_inc(&sk->bastet->hbm.reply_filter_cnt);
	}
	sock_put(sk);

	return err;
}
