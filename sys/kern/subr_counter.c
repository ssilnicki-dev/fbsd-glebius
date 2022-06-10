/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2012 Gleb Smirnoff <glebius@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysctl.h>
#include <vm/uma.h>

#define IN_SUBR_COUNTER_C
#include <sys/counter.h>

void
counter_u64_zero(counter_u64_t c)
{

	counter_u64_zero_inline(c);
}

uint64_t
counter_u64_fetch(counter_u64_t c)
{

	return (counter_u64_fetch_inline(c));
}

counter_u64_t
counter_u64_alloc(int flags)
{

	return (uma_zalloc_pcpu(pcpu_zone_8, flags | M_ZERO));
}

void
counter_u64_free(counter_u64_t c)
{

	uma_zfree_pcpu(pcpu_zone_8, c);
}

int
sysctl_handle_counter_u64(SYSCTL_HANDLER_ARGS)
{
	uint64_t out;
	int error;

	out = counter_u64_fetch(*(counter_u64_t *)arg1);

	error = SYSCTL_OUT(req, &out, sizeof(uint64_t));

	if (error || !req->newptr)
		return (error);

	/*
	 * Any write attempt to a counter zeroes it.
	 */
	counter_u64_zero(*(counter_u64_t *)arg1);

	return (0);
}

int
sysctl_handle_counter_u64_array(SYSCTL_HANDLER_ARGS)
{
	uint64_t *out;
	int error;

	out = malloc(arg2 * sizeof(uint64_t), M_TEMP, M_WAITOK);
	for (int i = 0; i < arg2; i++)
		out[i] = counter_u64_fetch(((counter_u64_t *)arg1)[i]);

	error = SYSCTL_OUT(req, out, arg2 * sizeof(uint64_t));
	free(out, M_TEMP);

	if (error || !req->newptr)
		return (error);

	/*
	 * Any write attempt to a counter zeroes it.
	 */
	for (int i = 0; i < arg2; i++)
		counter_u64_zero(((counter_u64_t *)arg1)[i]);

	return (0);
}

/*
 * MP-friendly version of ppsratecheck().
 *
 * Returns non-negative if we are in the rate, negative otherwise.
 *  0 - rate limit not reached.
 * -1 - rate limit reached.
 * >0 - rate limit was reached before, and was just reset. The return value
 *      is number of events since last reset.
 */
int64_t
counter_ratecheck(struct counter_rate *cr, int64_t limit)
{
	int64_t val;
	int now;

	val = cr->cr_over;
	now = ticks;

	if ((u_int)(now - cr->cr_ticks) >= hz) {
		/*
		 * Time to clear the structure, we are in the next second.
		 * First try unlocked read, and then proceed with atomic.
		 */
		if ((cr->cr_lock == 0) &&
		    atomic_cmpset_acq_int(&cr->cr_lock, 0, 1)) {
			/*
			 * Check if other thread has just went through the
			 * reset sequence before us.
			 */
			if ((u_int)(now - cr->cr_ticks) >= hz) {
				val = counter_u64_fetch(cr->cr_rate);
				counter_u64_zero(cr->cr_rate);
				cr->cr_over = 0;
				cr->cr_ticks = now;
				if (val <= limit)
					val = 0;
			}
			atomic_store_rel_int(&cr->cr_lock, 0);
		} else
			/*
			 * We failed to lock, in this case other thread may
			 * be running counter_u64_zero(), so it is not safe
			 * to do an update, we skip it.
			 */
			return (val);
	}

	counter_u64_add(cr->cr_rate, 1);
	if (cr->cr_over != 0)
		return (-1);
	if (counter_u64_fetch(cr->cr_rate) > limit)
		val = cr->cr_over = -1;

	return (val);
}

void
counter_u64_sysinit(void *arg)
{
	counter_u64_t *cp;

	cp = arg;
	*cp = counter_u64_alloc(M_WAITOK);
}

void
counter_u64_sysuninit(void *arg)
{
	counter_u64_t *cp;

	cp = arg;
	counter_u64_free(*cp);
}



/*
 * Imprecise but cheap resource management.
 */
int
counter_fo_init(struct counter_fo *c, uint64_t limit, uint64_t precision,
    int flags)
{
	int i;

	KASSERT(precision >= mp_ncpus,
	    ("%s: impossible precision %ju with ncpus %d", __func__,
	    (uintmax_t )precision, mp_ncpus));
	KASSERT(limit == 0 || precision < limit / 2,
	    ("%s: impossible precision %ju with limit %ju", __func__,
	    (uintmax_t )precision, (uintmax_t )limit));

	c->cf_counter = uma_zalloc_pcpu(pcpu_zone_64, flags);
	if (c->cf_counter == NULL)
		return (ENOMEM);
	mtx_init(&c->cf_mtx, "counter_fo", NULL, MTX_DEF | MTX_NEW);
	c->cf_budget = precision / mp_ncpus;
	c->cf_flags = 0;
	if ((c->cf_pool = limit) > 0) {
		CPU_FOREACH(i) {
			c->cf_pool -= c->cf_budget / 2;
			counter_u64_write_one(c->cf_counter,
			    c->cf_budget / 2, i);
		}
	} else
		counter_u64_zero(c->cf_counter);

	return (0);
}

void
counter_fo_fini(struct counter_fo *c)
{

	uma_zfree_pcpu(pcpu_zone_64, c->cf_counter);
	mtx_destroy(&c->cf_mtx);
}

uint64_t
counter_fo_fetchall(struct counter_fo *c)
{

	return (counter_u64_fetch(c->cf_counter) + c->cf_pool);
}

/*
 * Add to counter_fo.  Shall never fail, be the val positive or negative.
 *
 * If used with negative 'val', the assertion is that counter_fo pool + pcpu
 * sum is over 'val'.  The function may overcommit individial percpu counter.
 */
void
counter_fo_add(struct counter_fo *c, int64_t val, int flags)
{
	int64_t *valp;

	if ((val > 0 && val > c->cf_budget) || -val > c->cf_budget)  {
		if ((flags & CFO_NOBLOCK) == 0)
			mtx_lock(&c->cf_mtx);
		else if (!mtx_trylock(&c->cf_mtx)) {
			counter_u64_add(c->cf_counter, val);
			return;
		}
		c->cf_pool += val;
		goto wakeup;
	}

	val = (int64_t )counter_u64_fetchadd(c->cf_counter, val);
	if (val > 0 && val <= c->cf_budget)
		return;
	/*
	 * Try to sync with the shared pool.
	 */
	if ((flags & CFO_NOBLOCK) == 0)
		mtx_lock(&c->cf_mtx);
	else if (!mtx_trylock(&c->cf_mtx))
		return;
	critical_enter();
	valp = zpcpu_get(c->cf_counter);
	if (*valp > 0 && *valp <= c->cf_budget) {
		/*
		 * Migrated (or raced) and there is something left in
		 * the other CPU budget.  We will pretend that we took
		 * from there.
		 */
		critical_exit();
		mtx_unlock(&c->cf_mtx);
		return;
	}

	*valp += c->cf_pool;
	if (*valp > (int64_t )c->cf_budget / 2) {
		c->cf_pool = *valp - c->cf_budget / 2;
		*valp = c->cf_budget / 2;
		critical_exit();
wakeup:
		if (c->cf_flags & CFO_WAITERS) {
			c->cf_flags &= ~CFO_WAITERS;
			wakeup(c);
		}
	} else {
		critical_exit();
		c->cf_pool = 0;
	}
	mtx_unlock(&c->cf_mtx);
}

/*
 * Reduce counter_fo by val, that isn't guaranteed to be over 'val'.
 *
 * May fail if either CFO_NOBLOCK or CFO_NOSLEEP specified, otherwise
 * would sleep.
 *
 * The function should work with negative val, increasing the counter,
 * but that doesn't make any sense, lighter counter_fo_add() should be
 * used instead.
 */
bool
counter_fo_get(struct counter_fo *c, int64_t val, int flags, char *wmesg)
{
	int64_t new, *valp;

	if (val > c->cf_budget)  {
		if ((flags & CFO_NOBLOCK) == 0)
			mtx_lock(&c->cf_mtx);
		else if (!mtx_trylock(&c->cf_mtx))
			return (false);
restart1:
		if (c->cf_pool >= val) {
			c->cf_pool -= val;
			mtx_unlock(&c->cf_mtx);
			return (true);
		} else if (flags & CFO_NOSLEEP) {
			mtx_unlock(&c->cf_mtx);
			return (false);
		} else {
			c->cf_flags |= CFO_WAITERS;
			(void )mtx_sleep(c, &c->cf_mtx, PVM, wmesg, 0);
			goto restart1;
		}
	}


	new = (int64_t )counter_u64_fetchadd(c->cf_counter, -val);
	if (new >= 0)
		return (true);
	/*
	 * Try to take from shared pool.
	 */
	if ((flags & CFO_NOBLOCK) == 0)
		mtx_lock(&c->cf_mtx);
	else if (!mtx_trylock(&c->cf_mtx)) {
		counter_u64_add(c->cf_counter, val);
		return (false);
	}
restart2:
	critical_enter();
	valp = zpcpu_get(c->cf_counter);
	if (*valp >= 0) {
		/*
		 * Migrated (or raced) and there is something left in
		 * the other CPU budget.  We will pretend that we took
		 * from there.
		 */
		critical_exit();
		mtx_unlock(&c->cf_mtx);
	} else if (c->cf_pool > c->cf_budget / 2) {
		c->cf_pool -= c->cf_budget / 2;
		mtx_unlock(&c->cf_mtx);
		*valp += c->cf_budget / 2;
		critical_exit();
	} else if (c->cf_pool >= val) {
		c->cf_pool -= val;
		mtx_unlock(&c->cf_mtx);
		*valp += val;
		critical_exit();
	} else {
		/* Depleted. Put increment back and fail or sleep. */
		*valp += val;
		critical_exit();
		if (flags & CFO_NOSLEEP) {
			mtx_unlock(&c->cf_mtx);
			return (false);
		}
		c->cf_flags |= CFO_WAITERS;
		(void )mtx_sleep(c, &c->cf_mtx, PVM, wmesg, 0);
		new = (int64_t )counter_u64_fetchadd(c->cf_counter, -val);
		if (new < 0)
			goto restart2;
		mtx_unlock(&c->cf_mtx);
	}

	return (true);
}
