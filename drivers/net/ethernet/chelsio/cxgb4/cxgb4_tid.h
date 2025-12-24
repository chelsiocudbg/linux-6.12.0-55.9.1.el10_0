/*
 * This file is part of the Chelsio T4/T5/T6 Ethernet driver.
 *
 * Copyright (C) 2021 Chelsio Communications.  All rights reserved.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the LICENSE file included in this
 * release for licensing terms and conditions.
 */

#ifndef __CXGB4_TID_H__
#define __CXGB4_TID_H__
//BEGIN -----------------------old changes for comptability with other drivers --------------BEGIN
struct serv_entry {
        void *data;
};

union aopen_entry {
        void *data;
        union aopen_entry *next;
};

struct eotid_entry {
        void *data;
};

/*
 * Holds the size, base address, free list start, etc of the TID, server TID,
 * and active-open TID tables.  The tables themselves are allocated dynamically.
 */
struct tid_info {
        void **tid_tab;
        unsigned int tid_base;
        unsigned int ntids;

        struct serv_entry *stid_tab;
        unsigned long *stid_bmap;
        unsigned int nstids;
        unsigned int stid_base;

        unsigned int nhash;
        unsigned int hash_base;

        union aopen_entry *atid_tab;
        unsigned int natids;
        unsigned int atid_base;

        struct filter_entry *hpftid_tab;
        unsigned long *hpftid_bmap;
        unsigned int nhpftids;
        unsigned int hpftid_base;

        struct filter_entry *ftid_tab;
        unsigned long *ftid_bmap;
        unsigned int nftids;
        unsigned int ftid_base;
        unsigned int aftid_base;
        unsigned int aftid_end;
        /* Server filter region */
        unsigned int sftid_base;
        unsigned int nsftids;

        spinlock_t atid_lock ____cacheline_aligned_in_smp;
        union aopen_entry *afree;
        unsigned int atids_in_use;

        spinlock_t stid_lock;
        unsigned int stids_in_use;
        unsigned int v6_stids_in_use;
        unsigned int sftids_in_use;

        /* ETHOFLD range */
        struct eotid_entry *eotid_tab;
        unsigned long *eotid_bmap;
        unsigned int eotid_base;
        unsigned int neotids;

        /* TIDs in the TCAM */
        atomic_t tids_in_use;
        /* TIDs in the HASH */
        atomic_t hash_tids_in_use;
        atomic_t conns_in_use;
        /* ETHOFLD TIDs used for rate limiting */
        atomic_t eotids_in_use;

        /* lock for setting/clearing filter bitmap */
        spinlock_t ftid_lock;

        unsigned int tc_hash_tids_max_prio;
};

static inline void *lookup_tid(const struct tid_info *t, unsigned int tid)
{
       tid -= t->tid_base;
       return tid < t->ntids ? t->tid_tab[tid] : NULL;
}

static inline void *lookup_atid(const struct tid_info *t, unsigned int atid)
{
        return atid < t->natids ? t->atid_tab[atid].data : NULL;
}

static inline void *lookup_stid(const struct tid_info *t, unsigned int stid)
{
       /* Is it a server filter TID? */
       if (t->nsftids && (stid >= t->sftid_base)) {
               stid -= t->sftid_base;
               stid += t->nstids;
       } else {
               stid -= t->stid_base;
       }

       return stid < (t->nstids + t->nsftids) ? t->stid_tab[stid].data : NULL;
}

static inline void cxgb4_insert_tid(struct tid_info *t, void *data,
                                   unsigned int tid, unsigned short family)
{
       t->tid_tab[tid - t->tid_base] = data;
       if (t->hash_base && (tid >= t->hash_base)) {
               if (family == AF_INET6)
                       atomic_add(2, &t->hash_tids_in_use);
               else
                       atomic_inc(&t->hash_tids_in_use);
       } else {
               if (family == AF_INET6)
                       atomic_add(2, &t->tids_in_use);
               else
                       atomic_inc(&t->tids_in_use);
       }
       atomic_inc(&t->conns_in_use);
}

int cxgb4_alloc_atid(struct tid_info *t, void *data);
int cxgb4_alloc_stid(struct tid_info *t, int family, void *data);
int cxgb4_alloc_sftid(struct tid_info *t, int family, void *data);
void cxgb4_free_atid(struct tid_info *t, unsigned int atid);
void cxgb4_free_stid(struct tid_info *t, unsigned int stid, int family);
void cxgb4_remove_tid(struct tid_info *t, u16 qid, unsigned int tid,
                      unsigned short family);
//END -----------------------old changes for comptability with other drivers --------------END
struct cxgb4_tid_info_xarray {
       struct xarray tid_tab;
       unsigned long *bitmap;
       u32 start;
       u32 size;
       u32 in_use;
       u32 range_in_use;
       u8 max_range;
};

/* Holds the size, base address, data, etc. of the various TIDs in
 * hardware.
 */
struct cxgb4_tid_info {
       struct cxgb4_tid_info_xarray tids;
       struct cxgb4_tid_info_xarray atids;
       struct cxgb4_tid_info_xarray hpftids;
       struct cxgb4_tid_info_xarray ftids;
       struct cxgb4_tid_info_xarray hashtids;
       struct cxgb4_tid_info_xarray hashcoll_tids;
       struct cxgb4_tid_info_xarray stids;
       struct cxgb4_tid_info_xarray uotids;
       struct cxgb4_tid_info_xarray sftids;

       u64 tid_release_head;
       unsigned int tc_hash_tids_max_prio;
       spinlock_t tid_release_lock; /* Lock to access TID release list */
       struct work_struct tid_release_task;
};

unsigned int cxgb4_atid_in_use(struct adapter *adap);
void *cxgb4_atid_lookup(struct adapter *adap, u32 atid);
int cxgb4_atid_alloc(struct adapter *adap, void *data);
void cxgb4_atid_free(struct adapter *adap, u32 atid);

unsigned int cxgb4_tid_in_use(struct adapter *adap);
bool cxgb4_tid_out_of_range(struct adapter *adap, u32 tid);
void *cxgb4_tid_lookup(struct adapter *adap, u32 tid);
int cxgb4_tid_insert(struct adapter *adap, u32 tid, void *data, bool range);
void cxgb4_tid_remove(struct adapter *adap, u16 ctrlq_idx, u32 tid, bool range);

bool cxgb4_hashtid_out_of_range(struct adapter *adap, u32 hashtid);
void *cxgb4_hashtid_lookup(struct adapter *adap, u32 hashtid);
int cxgb4_hashtid_insert(struct adapter *adap, u32 hashtid, void *data,
                        bool range);
void cxgb4_hashtid_remove(struct adapter *adap, u16 ctrlq_idx, u32 hashtid,
                         bool range);

bool cxgb4_hpftid_out_of_range(struct adapter *adap, u32 hpftid);
int cxgb4_hpftid_alloc(struct adapter *adap, void *data, bool range);

bool cxgb4_ftid_out_of_range(struct adapter *adap, u32 ftid);
void *cxgb4_ftid_lookup(struct adapter *adap, u32 ftid);
int cxgb4_ftid_insert(struct adapter *adap, void *data, u32 ftid, bool range);
int cxgb4_ftid_alloc(struct adapter *adap, void *data, bool range);
void cxgb4_ftid_free(struct adapter *adap, u32 ftid, bool range);

bool cxgb4_sftid_out_of_range(struct adapter *adap, u32 sftid);
int cxgb4_sftid_alloc(struct adapter *adap, void *data, bool range);

void *cxgb4_stid_lookup(struct adapter *adap, u32 stid);
int cxgb4_stid_alloc(struct adapter *adap, void *data, bool range);
void cxgb4_stid_free(struct adapter *adap, u32 stid, bool range);

void *cxgb4_uotid_lookup(struct adapter *adap, u32 uotid);
int cxgb4_uotid_alloc(struct adapter *adap, void *data);
void cxgb4_uotid_free(struct adapter *adap, u32 uotid);

void cxgb4_tid_info_cleanup(struct adapter *adap);
int cxgb4_tid_info_init(struct adapter *adap,
                       const struct fw_caps_config_cmd *caps_cmd);
#endif /* __CXGB4_TID_H__ */
