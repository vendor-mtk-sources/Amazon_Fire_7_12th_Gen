/*
 * Obtain energy cost data from DT and populate relevant scheduler data
 * structures.
 *
 * Copyright (C) 2015 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define pr_fmt(fmt) "sched-energy: " fmt

#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/sched/topology.h>
#include <linux/sched/energy.h>
#include <linux/stddef.h>
#include <linux/arch_topology.h>
#include "energy_plus.h"
#include <linux/cpu.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>

#include "sched.h"

struct sched_group_energy *sge_array[NR_CPUS][NR_SD_LEVELS];

#ifndef CONFIG_MTK_UNIFY_POWER
static void free_resources(void)
{
	int cpu, sd_level;
	struct sched_group_energy *sge;

	for_each_possible_cpu(cpu) {
		for_each_possible_sd_level(sd_level) {
			sge = sge_array[cpu][sd_level];
			if (sge) {
				kfree(sge->cap_states);
				kfree(sge->idle_states);
				kfree(sge);
			}
		}
	}
}
static bool sge_ready;
static bool freq_energy_model;

void check_max_cap_vs_cpu_scale(int cpu, struct sched_group_energy *sge)
{
	unsigned long max_cap, cpu_scale;

	max_cap = sge->cap_states[sge->nr_cap_states - 1].cap;
	cpu_scale = topology_get_cpu_scale(NULL, cpu);

	if (max_cap == cpu_scale)
		return;

	pr_warn("CPU%d max energy model capacity=%ld != cpu_scale=%ld\n", cpu,
		max_cap, cpu_scale);
}

void init_sched_energy_costs(void)
{
	struct device_node *cn, *cp;
	struct capacity_state *cap_states;
	struct idle_state *idle_states;
	struct sched_group_energy *sge;
	const struct property *prop;
	int sd_level, i, nstates, cpu;
	const __be32 *val;

	for_each_possible_cpu(cpu) {
		cn = of_get_cpu_node(cpu, NULL);
		if (!cn) {
			pr_warn("CPU device node missing for CPU %d\n", cpu);
			return;
		}

		if (!of_find_property(cn, "sched-energy-costs", NULL)) {
			pr_warn("CPU device node has no sched-energy-costs\n");
			return;
		}
		/* Check if the energy model contains frequency/power values */
		if (of_find_property(cn, "freq-energy-model", NULL))
			freq_energy_model = true;

		for_each_possible_sd_level(sd_level) {
			cp = of_parse_phandle(cn, "sched-energy-costs", sd_level);
			if (!cp)
				break;

			prop = of_find_property(cp, "busy-cost-data", NULL);
			if (!prop || !prop->value) {
				pr_warn("No busy-cost data, skipping sched_energy init\n");
				goto out;
			}

			sge = kcalloc(1, sizeof(struct sched_group_energy),
				      GFP_NOWAIT);

			nstates = (prop->length / sizeof(u32)) / 2;
			cap_states = kcalloc(nstates,
					     sizeof(struct capacity_state),
					     GFP_NOWAIT);

			for (i = 0, val = prop->value; i < nstates; i++) {
				if (freq_energy_model) {
					/*
					 * Capacity values will be calculated later using
					 * frequency reported by OPP driver and cpu_uarch_scale
					 * values.
					 */
					cap_states[i].frequency = be32_to_cpup(val++);
					cap_states[i].cap = 0;
				} else {
					cap_states[i].frequency = 0;
					cap_states[i].cap = be32_to_cpup(val++);
				}
				cap_states[i].power = be32_to_cpup(val++);
			}

			sge->nr_cap_states = nstates;
			sge->cap_states = cap_states;

			prop = of_find_property(cp, "idle-cost-data", NULL);
			if (!prop || !prop->value) {
				pr_warn("No idle-cost data, skipping sched_energy init\n");
				goto out;
			}

			nstates = (prop->length / sizeof(u32));
			idle_states = kcalloc(nstates,
					      sizeof(struct idle_state),
					      GFP_NOWAIT);

			for (i = 0, val = prop->value; i < nstates; i++)
				idle_states[i].power = be32_to_cpup(val++);

			sge->nr_idle_states = nstates;
			sge->idle_states = idle_states;

			sge_array[cpu][sd_level] = sge;
		}
		if (!freq_energy_model)
			check_max_cap_vs_cpu_scale(cpu, sge_array[cpu][SD_LEVEL0]);
	}
	sge_ready = true;
	pr_info("Sched-energy-costs installed from DT\n");
	return;

out:
	free_resources();
}
#endif

#include "energy_plus.c"
