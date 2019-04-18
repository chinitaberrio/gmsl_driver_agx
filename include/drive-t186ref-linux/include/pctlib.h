/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 */
#ifndef INCLUDE_PCTLIB_H
#define INCLUDE_PCTLIB_H

#include <stdint.h>
#include <stdbool.h>
#include <pct.h>
#include <io-peripherals.h>

/**
 * @addtogroup pct_api
 * @brief API to parse PCT binary blob
 */

#ifndef uintptr_t
typedef unsigned long		uintptr_t;
#endif

// validates specified guest id
static inline bool  pct_is_valid_hyp_guest_nr(uint32_t guest_id) {
        return (guest_id < (uint32_t) GUEST_MAX_NR);
}

// initializes pctlib
bool pct_initialize(void);

// returns pointer to the calculated end of PCT structure.
uintptr_t  pct_get_end_struct(void);

// returns platform configuration pointer
const struct platform_conf *pct_platform_get_conf(void);

// returns guest configuration pointer for specified guest
const struct guest_conf *pct_guest_get_conf(uint32_t guest_id);

// updates guest smmu sid configuration for specified guest
bool pct_guest_set_smmu_sid_info(uint32_t guest_id,
	const struct arm_smmu_guest_sid_info *smmu_sid_info);

// returns ipa configuration pointer for specified ipa index
const struct ipa_mapping *pct_ipa_get_mapping(uint32_t ipa_index);

// returns gpio configuration pointer for specified gpio index
const struct guest_gpio_mapping *pct_gpio_get_mapping(uint32_t gpio_index);

// returns mempool configuration pointer for specified mempool index
const struct pct_mempool *pct_ivc_get_mempool_conf(uint32_t mempool_id);

// returns ivc queue configuration pointer for specified queue index
const struct pct_ivc_queue *pct_ivc_get_queue_conf(uint32_t queue_id);

// returns smmu hardware group configuration pointer for specified group index
const struct smmu_hwgroup_conf  *pct_plat_get_smmu_hwgroup_conf(
	uint32_t hw_group_index);

// returns hsp internal configuration pointer for specified index
const struct hsp_internal_res *pct_get_guest_hsp_internal_res(
	uint32_t guest_id, uint32_t hsp_index);

// returns vcpu configuration pointer for specified vcpu index
const struct vcpu_conf *pct_get_vcpu_conf(uint32_t vcpu);

// returns io mapping configuration pointer for specified mapping index
const struct guest_io_mapping *pct_io_get_mapping(int32_t periph_id);

// returns i2c configuration pointer for specified index
const struct guest_i2c_mapping *pct_i2c_get_mapping(uint32_t i2c_index);

// returns io owners information for specified io index
// owned_by_hyp may be NULL if not needed to be returned
void pct_io_get_owners(
	int32_t periph_id, uint32_t *owner_guest_id, bool *owned_by_hyp);

// returns io mapping owners information for specified io mapping
// owned_by_hyp may be NULL if not needed to be returned
void pct_io_mapping_get_owners(
	const struct guest_io_mapping *mapping,
	uint32_t *owner_guest_id,
	bool *owned_by_hyp);

// returns true if guest_id ownes io mapping information.
// returnds false otherwise.
bool pct_io_mapping_belongs_to_guest(
	const struct guest_io_mapping *mapping, uint32_t guest_id) ;

// returns true if guest_id ownes io index
// returnds false otherwise.
bool pct_io_belongs_to_guest(int32_t periph_id, uint32_t guest_id);

// returns true if hypervisor ownes io index
// returnds false otherwise.
bool pct_io_belongs_to_hypervisor(int32_t periph_id);

// returns gpio owner information for specified gpio index
// owned_by_hyp may be NULL if not needed to be returned
bool pct_gpio_mapping_belongs_to_guest(
	const struct guest_gpio_mapping *mapping, uint32_t guest_id) ;

// returns true if guest_id ownes gpio index
// returnds false otherwise.
bool pct_gpio_belongs_to_guest(
	uint32_t gpio_index, uint32_t guest_id);

// returns true if a guest ownes gpio mapping information. returns its id too.
// returnds false otherwise.
bool pct_gpio_mapping_to_owning_guest(
	const struct guest_gpio_mapping *mapping, uint32_t *ret_guest_id) ;

// returns true if a guest ownes gpio index. returns its id too.
// returnds false otherwise.
bool pct_gpio_to_owning_guest(
	uint32_t gpio_index, uint32_t *ret_guest_id);

// returns true if guest_id ownes ipa index
// returnds false otherwise.
bool pct_ipa_belongs_to_guest(uint32_t ipa_index, uint32_t guest_id);

// returns true if hypervisor ownes ipa index
// returnds false otherwise.
bool pct_ipa_belongs_to_hyp(uint32_t ipa_index);


// returns true if a guest ownes i2c slave. returns its id too.
// returnds false otherwise.
bool pct_i2c_slave_to_owning_guest(
		        uint32_t controller, uint32_t slave_addr,
			uint32_t *ret_guest_id);

void pct_hsp_assign(void);	// FIXME: create pct_hsp.h and more it there
void pct_se_init(void);		// FIXME: create pct_se.h and more it there
void pct_bpmp_init(void);	// FIXME: create pct_bpmp.h and more it there

uintptr_t pct_get_start(void);
uintptr_t pct_get_end(void);



#endif  // INCLUDE_PCTLIB_H
