// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/limits.h>

#include "ccu_gate.h"
#include "ccu_nkmp.h"

struct _ccu_nkmp {
	unsigned long	n, min_n, max_n;
	unsigned long	k, min_k, max_k;
	unsigned long	m, min_m, max_m;
	unsigned long	p, min_p, max_p;
	unsigned long	min_nk, max_nk;
	unsigned long	max_p_rate;
};

static unsigned long ccu_nkmp_calc_rate(unsigned long parent,
					unsigned long n, unsigned long k,
					unsigned long m, unsigned long p)
{
	u64 rate = parent;

	rate *= n * k;
	do_div(rate, m * p);

	return rate;
}

struct nkmp_best {
	unsigned long rate;
	unsigned long n, k, m, p;
};

static void ccu_nkmp_try(unsigned long parent, unsigned long rate,
			 unsigned long n, unsigned long k,
			 unsigned long m, unsigned long p,
			 struct nkmp_best *best)
{
	unsigned long tmp_rate = ccu_nkmp_calc_rate(parent, n, k, m, p);

	if (tmp_rate > rate)
		return;

	if ((rate - tmp_rate) < (rate - best->rate)) {
		best->rate = tmp_rate;
		best->n = n;
		best->k = k;
		best->m = m;
		best->p = p;
	}
}

static unsigned long ccu_nkmp_find_best(unsigned long parent, unsigned long rate,
					struct _ccu_nkmp *nkmp)
{
	struct nkmp_best best = {};
	unsigned long max_p = rate >= nkmp->max_p_rate ? 1 : nkmp->max_p;
	unsigned long _k, _m, _p;
	unsigned int approx;

	if (!parent)
		goto out;

	for (approx = 0; approx <= 1; approx++) {
		for (_m = nkmp->min_m; _m <= nkmp->max_m; _m++) {
			for (_p = nkmp->min_p; _p <= max_p; _p <<= 1) {
				u64 nk64 = (u64)rate * _m * _p;
				unsigned long nk_rem = do_div(nk64, parent);
				unsigned long nk = min_t(u64, nk64, nkmp->max_nk);

				if (!approx) {
					if (nk_rem != 0 || nk64 < nkmp->min_nk ||
					    nk64 > nkmp->max_nk)
						continue;
				} else if (nk64 + 1 < nkmp->min_nk) {
					/*
					 * Even the n+1 candidate (N*K ==
					 * min_nk) can't reach a rate <= the
					 * requested one when the target N*K
					 * is below min_nk - 1.
					 */
					continue;
				}

				for (_k = nkmp->min_k; _k <= nkmp->max_k; _k++) {
					unsigned long hi, _n, cand;

					if (!approx) {
						u64 n64 = nk64;
						unsigned long n_rem = do_div(n64, _k);

						if (n_rem != 0 ||
						    n64 < nkmp->min_n ||
						    n64 > nkmp->max_n)
							continue;

						nkmp->n = n64;
						nkmp->k = _k;
						nkmp->m = _m;
						nkmp->p = _p;
						return ccu_nkmp_calc_rate(parent, n64,
									  _k, _m, _p);
					}

					/*
					 * Highest n satisfying both the n
					 * range and the N*K limit.
					 */
					hi = min(nkmp->max_n, nkmp->max_nk / _k);
					_n = min(nk / _k, hi);

					/*
					 * The requested rate is often a
					 * truncation of the exactly achievable
					 * rate, so the best n can be one above
					 * the truncated analytic value.
					 */
					for (cand = _n; cand <= min(_n + 1, hi); cand++) {
						if (cand < nkmp->min_n ||
						    cand * _k < nkmp->min_nk)
							continue;

						ccu_nkmp_try(parent, rate, cand,
							     _k, _m, _p, &best);
					}
				}
			}
		}
	}

out:
	nkmp->n = best.n;
	nkmp->k = best.k;
	nkmp->m = best.m;
	nkmp->p = best.p;

	return best.rate;
}

static void ccu_nkmp_disable(struct clk_hw *hw)
{
	struct ccu_nkmp *nkmp = hw_to_ccu_nkmp(hw);

	return ccu_gate_helper_disable(&nkmp->common, nkmp->enable);
}

static int ccu_nkmp_enable(struct clk_hw *hw)
{
	struct ccu_nkmp *nkmp = hw_to_ccu_nkmp(hw);

	return ccu_gate_helper_enable(&nkmp->common, nkmp->enable);
}

static int ccu_nkmp_is_enabled(struct clk_hw *hw)
{
	struct ccu_nkmp *nkmp = hw_to_ccu_nkmp(hw);

	return ccu_gate_helper_is_enabled(&nkmp->common, nkmp->enable);
}

static unsigned long ccu_nkmp_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct ccu_nkmp *nkmp = hw_to_ccu_nkmp(hw);
	unsigned long n, m, k, p, rate;
	u32 reg;

	reg = readl(nkmp->common.base + nkmp->common.reg);

	n = reg >> nkmp->n.shift;
	n &= (1 << nkmp->n.width) - 1;
	n += nkmp->n.offset;
	if (!n)
		n++;

	k = reg >> nkmp->k.shift;
	k &= (1 << nkmp->k.width) - 1;
	k += nkmp->k.offset;
	if (!k)
		k++;

	m = reg >> nkmp->m.shift;
	m &= (1 << nkmp->m.width) - 1;
	m += nkmp->m.offset;
	if (!m)
		m++;

	p = reg >> nkmp->p.shift;
	p &= (1 << nkmp->p.width) - 1;

	rate = ccu_nkmp_calc_rate(parent_rate, n, k, m, 1 << p);
	if (nkmp->common.features & CCU_FEATURE_FIXED_POSTDIV)
		rate /= nkmp->fixed_post_div;

	return rate;
}

static int ccu_nkmp_determine_rate(struct clk_hw *hw,
				   struct clk_rate_request *req)
{
	struct ccu_nkmp *nkmp = hw_to_ccu_nkmp(hw);
	struct _ccu_nkmp _nkmp;

	if (nkmp->common.features & CCU_FEATURE_FIXED_POSTDIV)
		req->rate *= nkmp->fixed_post_div;

	if (nkmp->max_rate && req->rate > nkmp->max_rate) {
		req->rate = nkmp->max_rate;
		if (nkmp->common.features & CCU_FEATURE_FIXED_POSTDIV)
			req->rate /= nkmp->fixed_post_div;
		return 0;
	}

	_nkmp.min_n = nkmp->n.min ?: 1;
	_nkmp.max_n = nkmp->n.max ?: 1 << nkmp->n.width;
	_nkmp.min_k = nkmp->k.min ?: 1;
	_nkmp.max_k = nkmp->k.max ?: 1 << nkmp->k.width;
	_nkmp.min_m = 1;
	_nkmp.max_m = nkmp->m.max ?: 1 << nkmp->m.width;
	_nkmp.min_p = 1;
	_nkmp.max_p = nkmp->p.max ?: 1 << ((1 << nkmp->p.width) - 1);
	_nkmp.min_nk = nkmp->min_nk;
	_nkmp.max_nk = nkmp->max_nk ?: UINT_MAX;
	_nkmp.max_p_rate = nkmp->max_p_rate ?: UINT_MAX;

	req->rate = ccu_nkmp_find_best(req->best_parent_rate, req->rate,
				       &_nkmp);

	if (nkmp->common.features & CCU_FEATURE_FIXED_POSTDIV)
		req->rate = req->rate / nkmp->fixed_post_div;

	return 0;
}

static int ccu_nkmp_set_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long parent_rate)
{
	struct ccu_nkmp *nkmp = hw_to_ccu_nkmp(hw);
	u32 n_mask = 0, k_mask = 0, m_mask = 0, p_mask = 0;
	struct _ccu_nkmp _nkmp;
	unsigned long flags;
	u32 reg;

	if (nkmp->common.features & CCU_FEATURE_FIXED_POSTDIV)
		rate = rate * nkmp->fixed_post_div;

	_nkmp.min_n = nkmp->n.min ?: 1;
	_nkmp.max_n = nkmp->n.max ?: 1 << nkmp->n.width;
	_nkmp.min_k = nkmp->k.min ?: 1;
	_nkmp.max_k = nkmp->k.max ?: 1 << nkmp->k.width;
	_nkmp.min_m = 1;
	_nkmp.max_m = nkmp->m.max ?: 1 << nkmp->m.width;
	_nkmp.min_p = 1;
	_nkmp.max_p = nkmp->p.max ?: 1 << ((1 << nkmp->p.width) - 1);
	_nkmp.min_nk = nkmp->min_nk;
	_nkmp.max_nk = nkmp->max_nk ?: UINT_MAX;
	_nkmp.max_p_rate = nkmp->max_p_rate ?: UINT_MAX;

	ccu_nkmp_find_best(parent_rate, rate, &_nkmp);

	/*
	 * If width is 0, GENMASK() macro may not generate expected mask (0)
	 * as it falls under undefined behaviour by C standard due to shifts
	 * which are equal or greater than width of left operand. This can
	 * be easily avoided by explicitly checking if width is 0.
	 */
	if (nkmp->n.width)
		n_mask = GENMASK(nkmp->n.width + nkmp->n.shift - 1,
				 nkmp->n.shift);
	if (nkmp->k.width)
		k_mask = GENMASK(nkmp->k.width + nkmp->k.shift - 1,
				 nkmp->k.shift);
	if (nkmp->m.width)
		m_mask = GENMASK(nkmp->m.width + nkmp->m.shift - 1,
				 nkmp->m.shift);
	if (nkmp->p.width)
		p_mask = GENMASK(nkmp->p.width + nkmp->p.shift - 1,
				 nkmp->p.shift);

	spin_lock_irqsave(nkmp->common.lock, flags);

	reg = readl(nkmp->common.base + nkmp->common.reg);
	reg &= ~(n_mask | k_mask | m_mask | p_mask);

	reg |= ((_nkmp.n - nkmp->n.offset) << nkmp->n.shift) & n_mask;
	reg |= ((_nkmp.k - nkmp->k.offset) << nkmp->k.shift) & k_mask;
	reg |= ((_nkmp.m - nkmp->m.offset) << nkmp->m.shift) & m_mask;
	reg |= (ilog2(_nkmp.p) << nkmp->p.shift) & p_mask;

	writel(reg, nkmp->common.base + nkmp->common.reg);

	spin_unlock_irqrestore(nkmp->common.lock, flags);

	ccu_helper_wait_for_lock(&nkmp->common, nkmp->lock);

	return 0;
}

const struct clk_ops ccu_nkmp_ops = {
	.disable	= ccu_nkmp_disable,
	.enable		= ccu_nkmp_enable,
	.is_enabled	= ccu_nkmp_is_enabled,

	.recalc_rate	= ccu_nkmp_recalc_rate,
	.determine_rate = ccu_nkmp_determine_rate,
	.set_rate	= ccu_nkmp_set_rate,
};
EXPORT_SYMBOL_NS_GPL(ccu_nkmp_ops, "SUNXI_CCU");
