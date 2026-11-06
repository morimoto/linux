// SPDX-License-Identifier: 
//
// main.c
//
// Copyright (c) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define EINVAL	123

struct snd_soc_mod_top;
#define SND_SOC_MOD_NAME_SIZE 16
struct snd_soc_mod {
	char name[SND_SOC_MOD_NAME_SIZE];
	struct snd_soc_mod_top *top;

	struct snd_soc_mod **mod_in;
	int num_mod_in;
	struct snd_soc_mod **mod_out;
	int num_mod_out;

	void  *priv;
};

struct snd_soc_mod_top {
	char *base_name;
	int (*probe)(struct snd_soc_mod *mod);
	int (*remove)(struct snd_soc_mod *mod);
	struct snd_soc_mod *mods;
	int num_mods;
};

struct snd_soc_mod_graph {
	char *name;
	int (*probe)(struct snd_soc_mod_top *top);
	int (*remove)(struct snd_soc_mod_top *top);
	struct snd_soc_mod_top **tops;
	int num_tops;
};

#define snd_soc_mod_to_priv(mod)	(mod)->priv
#define for_each_snd_mod_tops(graph, i, top)		\
	for ((i) = 0;					\
	     ((i) < (graph)->num_tops) &&		\
		     ((top) = *((graph)->tops + i));	\
	     (i)++)
#define for_each_snd_mods(top, i, mod)  \
	for ((i) = 0;					\
	     ((i) < (top)->num_mods) &&			\
		     ((mod) = (top)->mods + i);		\
	     (i)++)
#define for_each_snd_mod_in(mod, i, in)  \
	for ((i) = 0;					\
	     ((i) < (mod)->num_mod_in) &&		\
		     ((in) = *((mod)->mod_in + i));	\
	     (i)++)
#define for_each_snd_mod_out(mod, i, out)		\
	for ((i) = 0;					\
	     ((i) < (mod)->num_mod_out) &&		\
		     ((out) = *((mod)->mod_out + i));	\
	     (i)++)

#define snd_soc_mod_id(mod) ((mod) - (mod)->top->mods)

int snd_soc_mod_graph_init(struct snd_soc_mod_graph *graph)
{
	struct snd_soc_mod_top *top;
	struct snd_soc_mod *mod;
	int i, j;

	/*
	 * setup name etc, first
	 */
	for_each_snd_mod_tops (graph, j, top) {
		for_each_snd_mods (top, i, mod) {
			mod->top = top;
			snprintf(mod->name, SND_SOC_MOD_NAME_SIZE, "%s%d", top->base_name, i);

			/*
			 * SND_SOC_MOD_DEFINE() might use size 0 array
			 * int this case, it has array pointer. set it as NULL here
			 * */
			if (!mod->num_mod_in)
				mod->mod_in = NULL;
			if (!mod->num_mod_out)
				mod->mod_out = NULL;
		}
	}

	/*
	 * graph sanity check
	 */
	for_each_snd_mod_tops (graph, j, top) {
		for_each_snd_mods (top, i, mod) {
			struct snd_soc_mod *in, *out;
			int t, s;

			for_each_snd_mod_in(mod, t, in) {
				for_each_snd_mod_out(in, s, out) {
					if (out == mod)
						goto found_out;
				}
				printf("%s not found\n", mod->name);
				return -EINVAL;
			found_out:
			}

			for_each_snd_mod_out(mod, t, out) {
				for_each_snd_mod_in(out, s, in) {
					if (in == mod)
						goto found_in;
				}
				printf("%s not found\n", mod->name);
				return -EINVAL;
			found_in:
			}
		}
	}
	return 0;
}

struct snd_soc_mod **snd_soc_mod_alloc_path(struct snd_soc_mod_graph *graph,
					    char **path, int num_path)
{
	struct snd_soc_mod *prev;
	struct snd_soc_mod **mods;

	mods = malloc(sizeof(mods) * num_path);
	if (!mods)
		goto err;

	prev = NULL;
	for (int i = 0; i < num_path; i++) {
		struct snd_soc_mod_top *top;
		struct snd_soc_mod *mod, *in, *out;
		int j, t;

		/*
		 * find mod from path name
		 * ex) "src0" => &src[0]
		 */
		for_each_snd_mod_tops(graph, j, top)
			for_each_snd_mods(top, t, mod)
				if (strncmp(path[i], mod->name, strlen(path[i])) == 0)
					goto found;

		printf("can't find path(%s) in graph(%s)\n", path[i], graph->name);
		goto free;
	found:
		/*
		 * check in/out
		 *
		 *         prev    mod
		 * NULL -> src0 -> ssi0 -> NULL
		 *           in            out
		 */

		/*
		 * In above sample case, src0 (= first mod) has no mod_in
		 */
		if (!mod->num_mod_in && !prev)
			goto out_check_ok;

		for_each_snd_mod_in(mod, j, in)
			if (in == prev)
				goto in_check_ok;

		printf("can't find %s in %s's mod_in \n", prev->name, path[i]);
		goto free;
	in_check_ok:
		/*
		 * In above sample case, ssi0 (= last mod) has no mod_out
		 */
		if (!mod->num_mod_out && (i + 1) == num_path)
			goto out_check_ok;

		for_each_snd_mod_out(prev, t, out)
			if (out == mod)
				goto out_check_ok;
		printf("can't find path(%s) in mod(%s)'s out \n", path[i], mod->name);
		goto free;
	out_check_ok:
		prev = mod;
		mods[i] = mod;
	}

	return mods;
free:
	free(mods);
err:
	return NULL;
}

#define SND_SOC_MOD_DEFINE(in, out) 		\
{						\
	.mod_in		= in,			\
	.num_mod_in	= ARRAY_SIZE(in),	\
	.mod_out	= out,			\
	.num_mod_out	= ARRAY_SIZE(out),	\
}

#define SND_SOC_MODS_BASE(a)	\
	.base_name = #a,	\
	.mods = a,		\
	.num_mods = ARRAY_SIZE(a)

#define SND_SOC_MOD_GRAPH_BASE(a)\
	.name = #a,		\
	.tops = a,		\
	.num_tops = ARRAY_SIZE(a)

/*
 * [SRC0] [MIX0] [DVC0] [SSI0]
 * [SRC1] [MIX1] [DVC1] [SSI1]
 * [SRC2]               [SSI2]
 * [SRC3]               [SSI3]
 */
#define NUM_SRC 4
#define NUM_MIX 2
#define NUM_DVC 2
#define NUM_SSI 4

extern struct snd_soc_mod src[NUM_SRC];
extern struct snd_soc_mod mix[NUM_MIX];
extern struct snd_soc_mod dvc[NUM_DVC];
extern struct snd_soc_mod ssi[NUM_SSI];

struct snd_soc_mod *src_in[] = {};
struct snd_soc_mod *src0_out[] = {&mix[0], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *src1_out[] = {&mix[0], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *src2_out[] = {&mix[1], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *src3_out[] = {&mix[1], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };

struct snd_soc_mod src[] = {
	SND_SOC_MOD_DEFINE(src_in, src0_out),
	SND_SOC_MOD_DEFINE(src_in, src1_out),
	SND_SOC_MOD_DEFINE(src_in, src2_out),
	SND_SOC_MOD_DEFINE(src_in, src3_out),
};

struct snd_soc_mod_top src_top = {
	SND_SOC_MODS_BASE(src),
};

struct snd_soc_mod *mix0_in[] = {&src[0], &src[1]};
struct snd_soc_mod *mix1_in[] = {&src[2], &src[3]};
struct snd_soc_mod *mix0_out[] = {&dvc[0]};
struct snd_soc_mod *mix1_out[] = {&dvc[1]};

struct snd_soc_mod mix[] = {
	SND_SOC_MOD_DEFINE(mix0_in, mix0_out),
	SND_SOC_MOD_DEFINE(mix1_in, mix1_out),
};

struct snd_soc_mod_top mix_top = {
	SND_SOC_MODS_BASE(mix),
};

struct snd_soc_mod *dvc0_in[] = {&mix[0]};
struct snd_soc_mod *dvc1_in[] = {&mix[1]};
struct snd_soc_mod *dvc0_out[] = {&ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *dvc1_out[] = {&ssi[0], &ssi[1], &ssi[2], &ssi[3] };

struct snd_soc_mod dvc[] = {
	SND_SOC_MOD_DEFINE(dvc0_in, dvc0_out),
	SND_SOC_MOD_DEFINE(dvc1_in, dvc1_out),
};

struct snd_soc_mod_top dvc_top = {
	SND_SOC_MODS_BASE(dvc),
};

struct snd_soc_mod *ssi0_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};
struct snd_soc_mod *ssi1_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};
struct snd_soc_mod *ssi2_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};
struct snd_soc_mod *ssi3_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};

struct snd_soc_mod *ssi_out[] = {};

struct snd_soc_mod ssi[] = {
	SND_SOC_MOD_DEFINE(ssi0_in, ssi_out),
	SND_SOC_MOD_DEFINE(ssi1_in, ssi_out),
	SND_SOC_MOD_DEFINE(ssi2_in, ssi_out),
	SND_SOC_MOD_DEFINE(ssi3_in, ssi_out),
};

struct snd_soc_mod_top ssi_top = {
	SND_SOC_MODS_BASE(ssi),
};

struct snd_soc_mod_top *my_graph[] = {
	&src_top,
	&mix_top,
	&dvc_top,
	&ssi_top,
};

struct snd_soc_mod_graph graph = {
	SND_SOC_MOD_GRAPH_BASE(my_graph),
};

//char *my_path[] = {"src0", "mix1", "dvc0", "ssi3"};
//char *my_path[]   = {"src0", "ssi3"};
char *my_path[]   = {"src3", "mix1", "dvc1", "ssi0"};

void main(void)
{
	struct snd_soc_mod *mod;
	struct snd_soc_mod **path;
	int i, ret;

	ret = snd_soc_mod_graph_init(&graph);
	if (ret < 0)
		return;

	path = snd_soc_mod_alloc_path(&graph, my_path, ARRAY_SIZE(my_path));
	if (!path) {
		printf("error\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(my_path); i++)
		printf("--%s\n", path[i]->name);
}
