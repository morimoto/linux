/* SPDX-License-Identifier: GPL-2.0-only
 *
 * soc-test-hack.h
 *
 **************** NOTE *******************
 * THIS IS FOR TEST PURPOSE !
 * DON'T USE THIS HEADER IN NORMAL DRIVER
 *****************************************
 *
 * Copyright (c) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_TEST_HACK_H
#define __SOC_TEST_HACK_H

#if IS_ENABLED(CONFIG_SND_SOC_TEST_HACK)
#include <kunit/test.h>

void test_hack_component_setup(struct snd_soc_component *component,
			       struct snd_soc_card *card,
			       struct regmap *regmap);
const char *test_hack_component_setup_name_prefix(struct snd_soc_component *component,
						  const char *name_prefix);

#endif /* CONFIG_SND_SOC_TEST_HACK */
#endif /* __SOC_TEST_HACK_H */
