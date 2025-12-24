/*
 * This file is part of the Chelsio T4/T5/T6 Ethernet driver for Linux.
 *
 * Copyright (C) 2021 Chelsio Communications.  All rights reserved.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the LICENSE file included in this
 * release for licensing terms and conditions.
 */

#include "cxgb4.h"
#include "t4_regs.h"
#include "t4_msg.h"
#include "t4fw_api.h"

#include "cxgb4_uld.h"
#include "cxgb4_filter.h"
static void cxgb4_tid_release_write(struct sk_buff *skb, u16 ctrlq_idx, u32 tid);

static int get_ipv6_max_range(struct adapter* adap)
{
	u8 ipv6_max_range = 2;
	u32 chip_ver = CHELSIO_CHIP_VERSION(adap->params.chip);
	u32 lconfig = t4_read_reg(adap, LE_DB_CONFIG_A);

	if (chip_ver > CHELSIO_T6 && lconfig & HASHEN_F)
		ipv6_max_range = 1;

	return ipv6_max_range;
}

//BEGIN -----------------------old changes for comptability with other drivers -----------BEGIN
/*
 * Queue a TID release request and if necessary schedule a work queue to
 * process it.
 */
static void cxgb4_queue_tid_release(struct tid_info *t, unsigned int ctrlq_idx,
                                   unsigned int tid)
{
       struct adapter *adap = container_of(t, struct adapter, tids);
       void **p = &t->tid_tab[tid - t->tid_base];

       spin_lock_bh(&adap->tid_release_lock);
       *p = adap->tid_release_head;
       /* Low 16 bits encode the Tx channel number */
       adap->tid_release_head = (void **)((uintptr_t)p | ctrlq_idx);
       if (!adap->tid_release_task_busy) {
               adap->tid_release_task_busy = true;
               queue_work(adap->workq, &adap->tid_release_task);
       }
       spin_unlock_bh(&adap->tid_release_lock);
}

/*
 * Allocate an active-open TID and set it to the supplied value.
 */
int cxgb4_alloc_atid(struct tid_info *t, void *data)
{
       int atid = -1;

       spin_lock_bh(&t->atid_lock);
       if (t->afree) {
               union aopen_entry *p = t->afree;

               atid = (p - t->atid_tab) + t->atid_base;
               t->afree = p->next;
               p->data = data;
               t->atids_in_use++;
       }
       spin_unlock_bh(&t->atid_lock);
       return atid;
}
EXPORT_SYMBOL(cxgb4_alloc_atid);

/*
 * Allocate a server TID and set it to the supplied value.
 */
int cxgb4_alloc_stid(struct tid_info *t, int family, void *data)
{
       int stid;

       spin_lock_bh(&t->stid_lock);
       if (family == PF_INET) {
               stid = find_first_zero_bit(t->stid_bmap, t->nstids);
               if (stid < t->nstids)
                       __set_bit(stid, t->stid_bmap);
               else
                       stid = -1;
       } else {
               stid = bitmap_find_free_region(t->stid_bmap, t->nstids, 1);
               if (stid < 0)
                       stid = -1;
       }
       if (stid >= 0) {
               t->stid_tab[stid].data = data;
               stid += t->stid_base;
               /* IPv6 requires max of 520 bits or 16 cells in TCAM
                * This is equivalent to 4 TIDs. With CLIP enabled it
                * needs 2 TIDs.
                */
               if (family == PF_INET6) {
                       t->stids_in_use += 2;
                       t->v6_stids_in_use += 2;
               } else {
                       t->stids_in_use++;
               }
       }
       spin_unlock_bh(&t->stid_lock);
       return stid;
}
EXPORT_SYMBOL(cxgb4_alloc_stid);

/* Allocate a server filter TID and set it to the supplied value.
 */
int cxgb4_alloc_sftid(struct tid_info *t, int family, void *data)
{
       int stid;

       spin_lock_bh(&t->stid_lock);
       if (family == PF_INET) {
               stid = find_next_zero_bit(t->stid_bmap,
                               t->nstids + t->nsftids, t->nstids);
               if (stid < (t->nstids + t->nsftids))
                       __set_bit(stid, t->stid_bmap);
               else
                       stid = -1;
       } else {
               stid = -1;
       }
       if (stid >= 0) {
               t->stid_tab[stid].data = data;
               stid -= t->nstids;
               stid += t->sftid_base;
               t->sftids_in_use++;
       }
       spin_unlock_bh(&t->stid_lock);
       return stid;
}
EXPORT_SYMBOL(cxgb4_alloc_sftid);

void cxgb4_free_atid(struct tid_info *t, unsigned int atid)
{
       union aopen_entry *p = &t->atid_tab[atid - t->atid_base];

       spin_lock_bh(&t->atid_lock);
       p->next = t->afree;
       t->afree = p;
       t->atids_in_use--;
       spin_unlock_bh(&t->atid_lock);
}
EXPORT_SYMBOL(cxgb4_free_atid);

/* Release a server TID.
 */
void cxgb4_free_stid(struct tid_info *t, unsigned int stid, int family)
{
       /* Is it a server filter TID? */
       if (t->nsftids && (stid >= t->sftid_base)) {
               stid -= t->sftid_base;
               stid += t->nstids;
       } else {
               stid -= t->stid_base;
       }

       spin_lock_bh(&t->stid_lock);
       if (family == PF_INET)
               __clear_bit(stid, t->stid_bmap);
       else
               bitmap_release_region(t->stid_bmap, stid, 1);
       t->stid_tab[stid].data = NULL;
       if (stid < t->nstids) {
               if (family == PF_INET6) {
                       t->stids_in_use -= 2;
                       t->v6_stids_in_use -= 2;
               } else {
                       t->stids_in_use--;
               }
       } else {
               t->sftids_in_use--;
       }

       spin_unlock_bh(&t->stid_lock);
}
EXPORT_SYMBOL(cxgb4_free_stid);

/*
 * Release a TID and inform HW.  If we are unable to allocate the release
 * message we defer to a work queue.
 */
void cxgb4_remove_tid(struct tid_info *t, u16 ctrlq_idx, unsigned int tid,
                      unsigned short family)
{
        struct adapter *adap = container_of(t, struct adapter, tids);
        struct sk_buff *skb;

        if (cxgb4_tid_out_of_range(adap, tid))
	{
                dev_err(adap->pdev_dev, "tid %d out of range\n", tid);
                return;
        }

        if (t->tid_tab[tid - adap->tids.tid_base]) {
                t->tid_tab[tid - adap->tids.tid_base] = NULL;
                atomic_dec(&t->conns_in_use);
                if (t->hash_base && (tid >= t->hash_base)) {
                        if (family == AF_INET6)
                                atomic_sub(2, &t->hash_tids_in_use);
                        else
                                atomic_dec(&t->hash_tids_in_use);
                } else {
                        if (family == AF_INET6)
                                atomic_sub(2, &t->tids_in_use);
                        else
                                atomic_dec(&t->tids_in_use);
                }
        }

        skb = alloc_skb(sizeof(struct cpl_tid_release), GFP_ATOMIC);
        if (likely(skb)) {
		cxgb4_tid_release_write(skb, ctrlq_idx, tid);
		cxgb4_sge_xmit_ctrl(adap->port[0], skb);

        } else
                cxgb4_queue_tid_release(t, ctrlq_idx, tid);
}
EXPORT_SYMBOL(cxgb4_remove_tid);

/*
 * Process the list of pending TID release requests.
 */
static void process_tid_release_list(struct work_struct *work)
{
       struct sk_buff *skb;
       struct adapter *adap;

       adap = container_of(work, struct adapter, tid_release_task);

       spin_lock_bh(&adap->tid_release_lock);
       while (adap->tid_release_head) {
               void **p = adap->tid_release_head;
               unsigned int ctrlq_idx = (uintptr_t)p & 0xffff;
               p = (void *)p - ctrlq_idx;

               adap->tid_release_head = *p;
               *p = NULL;
               spin_unlock_bh(&adap->tid_release_lock);

               while (!(skb = alloc_skb(sizeof(struct cpl_tid_release),
                                        GFP_KERNEL)))
                       schedule_timeout_uninterruptible(1);

	       cxgb4_tid_release_write(skb, ctrlq_idx, p - adap->tids.tid_tab);
	       cxgb4_sge_xmit_ctrl(adap->port[0], skb);
               spin_lock_bh(&adap->tid_release_lock);
       }
       adap->tid_release_task_busy = false;
       spin_unlock_bh(&adap->tid_release_lock);
}
//END -----------------------old changes for comptability with other drivers -----------END

unsigned int cxgb4_atid_in_use(struct adapter *adap)
{
       struct tid_info *t = &adap->tids;
       int ret;

       ret = t->atids_in_use;
       return ret;
}

static bool cxgb4_atid_out_of_range(struct adapter *adap, u32 atid)
{
       return atid < adap->tids.atid_base || atid >= (adap->tids.atid_base + adap->tids.natids);
}

void *cxgb4_atid_lookup(struct adapter *adap, u32 atid)
{
       if (cxgb4_atid_out_of_range(adap, atid))
               return NULL;

       return lookup_atid(&adap->tids, atid);
}

/*
 * upper-layer driver support
 */

int cxgb4_atid_alloc(struct adapter *adap, void *data)
{
       struct tid_info *t = &adap->tids;
       int ret;

       if (!t->natids)
               return -EOPNOTSUPP;

       ret = cxgb4_alloc_atid(t, data);
       return ret;
}

void cxgb4_atid_free(struct adapter *adap, u32 atid)
{
       struct tid_info *t = &adap->tids;

       if (cxgb4_atid_out_of_range(adap, atid))
               return;
       cxgb4_free_atid(t, atid);
}

unsigned int cxgb4_tid_in_use(struct adapter *adap)
{
       struct tid_info *t = &adap->tids;
       int ret;

       ret = atomic_read(&t->tids_in_use) +  get_ipv6_max_range(adap);
       return ret;
}

bool cxgb4_tid_out_of_range(struct adapter *adap, u32 tid)
{
       return tid < adap->tids.tid_base || tid >= (adap->tids.tid_base + adap->tids.ntids);
}

void *cxgb4_tid_lookup(struct adapter *adap, u32 tid)
{
       if (cxgb4_tid_out_of_range(adap, tid))
               return NULL;

       return lookup_tid(&adap->tids, tid);
}

int cxgb4_tid_insert(struct adapter *adap, u32 tid, void *data, bool range)
{
       struct tid_info *t = &adap->tids;

       if (cxgb4_tid_out_of_range(adap, tid))
               return -EOPNOTSUPP;

       if (range)
	       tid &= ~(get_ipv6_max_range(adap) - 1);

       cxgb4_insert_tid(t, data, tid, range);
       return 0;
}

/* Populate a TID_RELEASE WR.  Caller must properly size the skb.
 */
static void cxgb4_tid_release_write(struct sk_buff *skb, u16 ctrlq_idx, u32 tid)
{
       struct cpl_tid_release *req;

       /* make sure lld driver need to pass tid  mapped ctrlq_index */
       set_wr_txq(skb, CPL_PRIORITY_SETUP, ctrlq_idx);
       req = (struct cpl_tid_release *)__skb_put(skb, sizeof(*req));
       INIT_TP_WR_MIT_CPL(req, CPL_TID_RELEASE, tid);
}



/* Release a TID and inform HW.  If we are unable to allocate the release
 * message we defer to a work queue.
 */
void cxgb4_tid_remove(struct adapter *adap, u16 ctrlq_idx, u32 tid, bool range)
{
       struct tid_info *t = &adap->tids;

       if (cxgb4_tid_out_of_range(adap, tid))
               return;

       cxgb4_remove_tid(t, ctrlq_idx, tid, range);
}

static bool cxgb4_hash_tid_out_of_range(struct adapter *adap, u32 hashtid)
{
       return hashtid < adap->tids.hash_base || hashtid >= (adap->tids.hash_base + adap->tids.nhash);
}

bool cxgb4_hashtid_out_of_range(struct adapter *adap, u32 hashtid)
{
       return cxgb4_hash_tid_out_of_range(adap, hashtid);
}

void *cxgb4_hashtid_lookup(struct adapter *adap, u32 hashtid)
{
       if (cxgb4_hash_tid_out_of_range(adap, hashtid))
               return NULL;

       return cxgb4_tid_lookup(adap, hashtid);
}

int cxgb4_hashtid_insert(struct adapter *adap, u32 hashtid, void *data,
                        bool range)
{
       if (cxgb4_hash_tid_out_of_range(adap, hashtid))
               return -EOPNOTSUPP;

       if (range)
               hashtid &= ~(get_ipv6_max_range(adap) - 1);

       cxgb4_tid_insert(adap, hashtid, data, range);
       return 0;
}

void cxgb4_hashtid_remove(struct adapter *adap, u16 ctrlq_idx, u32 hashtid,
                         bool range)
{
       cxgb4_tid_remove(adap, ctrlq_idx, hashtid, range);
}

bool cxgb4_hpftid_out_of_range(struct adapter *adap, u32 hpftid)
{
	return hpftid < adap->tids.hpftid_base || hpftid >= (adap->tids.hpftid_base + adap->tids.nhpftids);
}

static void *cxgb4_hpftid_lookup(struct adapter *adap, u32 hpftid)
{
       return cxgb4_tid_lookup(adap, hpftid);
}

static int cxgb4_hpftid_insert(struct adapter *adap, void *data, u32 hpftid,
                              bool range)
{
       if (range)
               hpftid &= ~(2 - 1);

       cxgb4_tid_insert(adap, hpftid, data, range);
       return 0;
}

int cxgb4_hpftid_alloc(struct adapter *adap, void *data, bool range)
{
       return 0;
}

static void cxgb4_hpftid_free(struct adapter *adap, u32 hpftid, bool range)
{
       cxgb4_tid_remove(adap, 0, hpftid, range);
}

bool cxgb4_ftid_out_of_range(struct adapter *adap, u32 ftid)
{
	return ftid < adap->tids.ftid_base || ftid >= (adap->tids.ftid_base + adap->tids.nftids);
}

void *cxgb4_ftid_lookup(struct adapter *adap, u32 ftid)
{
       if (!cxgb4_hpftid_out_of_range(adap, ftid))
               return cxgb4_hpftid_lookup(adap, ftid);

       if (cxgb4_ftid_out_of_range(adap, ftid))
               return NULL;

       return cxgb4_tid_lookup(adap, ftid);
}

int cxgb4_ftid_insert(struct adapter *adap, void *data, u32 ftid, bool range)
{
       u32 chip_ver = CHELSIO_CHIP_VERSION(adap->params.chip);

       if (!cxgb4_hpftid_out_of_range(adap, ftid))
               return cxgb4_hpftid_insert(adap, data, ftid, range);

       if (cxgb4_ftid_out_of_range(adap, ftid))
               return -EOPNOTSUPP;

       if (range)
	       ftid &= ~((chip_ver > CHELSIO_T5 ? 2 : 4)- 1);

       cxgb4_tid_insert(adap, ftid, data, range);
       return 0;
}

int cxgb4_ftid_alloc(struct adapter *adap, void *data, bool range)
{
       return 0;
}

void cxgb4_ftid_free(struct adapter *adap, u32 ftid, bool range)
{

       if (!cxgb4_hpftid_out_of_range(adap, ftid)) {
               cxgb4_hpftid_free(adap, ftid, range);
               return;
       }

       if (cxgb4_ftid_out_of_range(adap, ftid))
               return;
       cxgb4_tid_remove(adap, 0, ftid, range);
}

bool cxgb4_sftid_out_of_range(struct adapter *adap, u32 sftid)
{
	return sftid < adap->tids.sftid_base || sftid >= (adap->tids.sftid_base + adap->tids.nsftids);
}

static void *cxgb4_sftid_lookup(struct adapter *adap, u32 sftid)
{
       return lookup_stid(&adap->tids, sftid);
}

int cxgb4_sftid_alloc(struct adapter *adap, void *data, bool range)
{
       struct tid_info *t = &adap->tids;
       int ret;

       if (!t->nsftids || range)
               return -EOPNOTSUPP;

       ret =  cxgb4_alloc_sftid(t, range, data);
       return ret;
}

static void cxgb4_sftid_free(struct adapter *adap, u32 sftid)
{
       struct tid_info *t = &adap->tids;

       cxgb4_free_stid(t, sftid, 0);
}

static bool cxgb4_stid_out_of_range(struct adapter *adap, u32 stid)
{

	return stid < adap->tids.stid_base || stid >= (adap->tids.stid_base + adap->tids.nstids);
}

void *cxgb4_stid_lookup(struct adapter *adap, u32 stid)
{
       if (!cxgb4_sftid_out_of_range(adap, stid))
               return cxgb4_sftid_lookup(adap, stid);

       if (cxgb4_stid_out_of_range(adap, stid))
               return NULL;

       return lookup_stid(&adap->tids, stid);
}

int cxgb4_stid_alloc(struct adapter *adap, void *data, bool range)
{
       struct tid_info *t = &adap->tids;
       int ret;

       if (!t->nstids)
               return -EOPNOTSUPP;

       ret = cxgb4_alloc_stid(t, range, data);
       return ret;
}

void cxgb4_stid_free(struct adapter *adap, u32 stid, bool range)
{
       struct tid_info *t = &adap->tids;

       if (!cxgb4_sftid_out_of_range(adap, stid)) {
               cxgb4_sftid_free(adap, stid);
               return;
       }

       if (cxgb4_stid_out_of_range(adap, stid))
               return;

       cxgb4_free_stid(t, stid, 0);
}


static int cxgb4_tid_query_params(struct adapter *adap,
                                 const struct fw_caps_config_cmd *caps_cmd)
{
       struct tid_info *t = &adap->tids;
       u32 tid_size, stid_start;
       u32 params[7], val[7];
       unsigned int chip_ver;
       u32 stid_size;
       int ret;

       chip_ver = CHELSIO_CHIP_VERSION(adap->params.chip);

       /* T6 TCAM can contain about 4 regions (Hi-Priority filter,
        * Active, Server and Normal priority filter regions).
        */
       if (chip_ver > CHELSIO_T5) {
               params[0] = FW_PARAM_PFVF(HPFILTER_START);
               params[1] = FW_PARAM_PFVF(HPFILTER_END);
               ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 2,
                                     params, val);
               if (ret < 0)
                       return ret;
               t->hpftid_base = val[0];
               t->nhpftids = val[1] - val[0] + 1;
       }

       params[0] = FW_PARAM_DEV(NTID);
       params[1] = FW_PARAM_PFVF(FILTER_START);
       params[2] = FW_PARAM_PFVF(FILTER_END);
       ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 3, params, val);
       if (ret < 0)
               return ret;
       tid_size = val[0];
       t->ftid_base = val[1];
       t->nftids = val[2] - val[1] + 1;

       params[0] = FW_PARAM_PFVF(SERVER_START);
       params[1] = FW_PARAM_PFVF(SERVER_END);
       ret = t4_query_params(adap, adap->mbox, adap->pf, 0, 2, params, val);
       if (ret < 0)
               return ret;
       stid_start = val[0];
       stid_size = val[1] - val[0] + 1;

       if ((caps_cmd->niccaps & htons(FW_CAPS_CONFIG_NIC_HASHFILTER)) &&
           chip_ver > CHELSIO_T4) {
               u32 lconfig, hbase, hconfig;

               if (cxgb4_hash_filter_config_verify(adap, !!caps_cmd->ofldcaps))
                       goto hashtids_done;

               lconfig = t4_read_reg(adap, LE_DB_CONFIG_A);
               if (lconfig & HASHEN_F) {
                       hconfig = t4_read_reg(adap, LE_DB_HASH_CONFIG_A);
                       if (chip_ver > CHELSIO_T5) {
                               hbase = t4_read_reg(adap,
                                                   T6_LE_DB_HASH_TID_BASE_A);
                               t->hash_base = hbase;
                               t->nhash = hconfig & 0xfffffU;
                       } else {
                               hbase = t4_read_reg(adap,
                                                   LE_DB_TID_HASHBASE_A);
                               t->hash_base = hbase;
                               t->nhash = (1 << HASHTIDSIZE_G(hconfig));
                       }

               }
       }

hashtids_done:
       if (!caps_cmd->ofldcaps)
               goto offload_tids_done;

       if (stid_size) {
               t->stid_base = stid_start;
               t->nstids = stid_size;
       }


       /* Setup server filter region. Divide the available filter
        * region into two parts. Regular filters get 1/3rd and server
        * filters get 2/3rd part. This is only enabled if workarond
        * path is enabled.
        * 1. For regular filters.
        * 2. Server filter: This are special filters which are used
        * to redirect SYN packets to offload queue.
        */
       if ((adap->flags & CXGB4_FW_OFLD_CONN) && !is_bypass(adap)) {
               u32 n_user_filters;
                       /* If we have invalid value in module-param then,
                        * use default value of 33% for user-filters.
                        */
                       n_user_filters = mult_frac(t->nftids, 33, 100);
               t->sftid_base = t->ftid_base + n_user_filters;
               t->nsftids = t->nftids - n_user_filters;
               /* Reserve last sftid for default-rule filter */
               t->nsftids--;
       }

offload_tids_done:

       if (tid_size) {
               t->tid_base = t->hpftid_base + t->hpftid_base;
               t->ntids = tid_size;
               t->atid_base = 0;
               /*
                * ATID field in CPL_ACT_OPEN_REQ is 24 bit wide of which atid
                * is 14 bit wide. So the CXGB4_MAX_ATIDS can be a most 2^14 - 1.
                * remaining 10 bits are used for rss_qid.
                */
               t->atid_base = min(t->ntids, CXGB4_MAX_ATIDS);
       }

       /* Now from t7,
        *--------------------------------------------------
        *      (ipv4)  (ipv6) (ipv6 with HASHEN &
        *                      EXTN_HASH_IPV4 set)
        * TCAM    1      2             2
        * HASH    1      2             1
        * --------------------------------------------------
        */
       return 0;
}


static int cxgb4_tid_info_xarray_init(unsigned int *tmp)
{
       return 0;
}

void cxgb4_tid_info_cleanup(struct adapter *adap)
{
       cxgb4_work_cancel(adap->workq, &adap->tid_release_task);
       memset(&adap->tids, 0, sizeof(adap->tids));
}

/* Allocate and initialize the TID tables.  Returns 0 on success.
 */
int cxgb4_tid_info_init(struct adapter *adap,
                       const struct fw_caps_config_cmd *caps_cmd)
{
       struct tid_info *t = &adap->tids;
       int ret;

       spin_lock_init(&adap->tid_release_lock);
       INIT_WORK(&adap->tid_release_task, process_tid_release_list);

       ret = cxgb4_tid_query_params(adap, caps_cmd);
       if (ret)
               goto out_free;

       ret = cxgb4_tid_info_xarray_init(&t->tid_base);
       if (ret)
               goto out_free;

       ret = cxgb4_tid_info_xarray_init(&t->atid_base);
       if (ret)
               goto out_free;

       ret = cxgb4_tid_info_xarray_init(&t->ftid_base);
       if (ret)
               goto out_free;

       ret = cxgb4_tid_info_xarray_init(&t->hpftid_base);
       if (ret)
               goto out_free;

       ret = cxgb4_tid_info_xarray_init(&t->hash_base);
       if (ret)
               goto out_free;


       ret = cxgb4_tid_info_xarray_init(&t->stid_base);
       if (ret)
               goto out_free;


       ret = cxgb4_tid_info_xarray_init(&t->sftid_base);
       if (ret)
               goto out_free;

       return 0;

out_free:
       cxgb4_tid_info_cleanup(adap);
       return ret;
}
