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

struct snd_soc_mod_ctr;
struct snd_soc_mod_graph;
struct snd_soc_mod_subpath;
struct snd_soc_mod_path;

/*
 * ex)
 *	[in] -->       --> [out]
 *	[in] --> [SRC] --> [out]
 *	[in] -->       --> [out]
 *	...                ...
 */
#define SND_SOC_MOD_NAME_SIZE 16
struct snd_soc_mod {
	char name[SND_SOC_MOD_NAME_SIZE];

	struct snd_soc_mod **mod_in;
	int num_mod_in;
	struct snd_soc_mod **mod_out;
	int num_mod_out;

	struct snd_soc_mod_ctr *ctr;
	void  *priv;
};
#define snd_soc_mod_to_priv(mod)	(mod)->priv

/*
 * ex)
 *	[SRC-ctr]
 *	 - [SRC0]
 *	 - [SRC1]
 *	 - [SRC2]
 *	   ...
 */
struct snd_soc_mod_ctr {
	char *base_name;
	int (*get_rate)(struct snd_soc_mod_subpath *path, struct snd_soc_mod *mod);
	struct snd_soc_mod *mods;
	int num_mods;

	struct snd_soc_mod_graph *graph;
	void  *priv;
};
#define snd_soc_mod_ctr_to_priv(ctr)	(ctr)->priv

/*
 * ex)
 *	[graph]
 *	 - [SRC-ctr]	- [MIX-ctr]	- [DVC-ctr]	- [SSI-ctr]
 *	  - [SRC0]	 - [MIX0]	 - [DVC0]	 - [SSI0]
 *	  - [SRC1]	 - [MIX1]	 - [DVC1]	 - [SSI1]
 *	  - [SRC2]					 - [SSI2]
 *	  - [SRC3]					 - [SSI3]
 *	    ...
 */
struct snd_soc_mod_graph {
	char *name;
	struct snd_soc_mod_ctr **ctrs;
	int num_ctrs;

	void  *priv;
};
#define snd_soc_mod_graph_to_priv(graph)	(graph)->priv

/*
 * ex)
 *	subpath
 *	 - [SRC2] - [MIX1] - [DVC1] - [SSI2]
 */
struct snd_soc_mod_subpath {
	struct snd_soc_mod **mods;
	int num_mods;

	void  *priv;
};
#define snd_soc_mod_subpath_to_priv(path)	(path)->priv

/*
 * ex)
 *	path
 *	 - [SRC2] - [MIX1] - [DVC1] - [SSI2]
 *	 - [SRC2] - [MIX1] - [DVC1] - [SSI3]
 *	 - ...
 *
 * Some vendor might want to use 1subpath : 1path,
 * some vendor might want to use Nsubpath : 1path,
 * but framework doesn't define details. Leave it to vendor
 */
struct snd_soc_mod_path {
	struct snd_soc_mod_subpath *current_subpath;
	struct snd_soc_mod_subpath **subpath;
	int num_subpath;

	void  *priv;
};
#define snd_soc_mod_path_to_priv(path)	(path)->priv

#define for_each_snd_mod_ctrs(graph, i, ctr)		\
	for ((i) = 0;					\
	     ((i) < (graph)->num_ctrs) &&		\
		     ((ctr) = *((graph)->ctrs + i));	\
	     (i)++)
#define for_each_snd_mods(ctr, i, mod)  \
	for ((i) = 0;					\
	     ((i) < (ctr)->num_mods) &&			\
		     ((mod) = (ctr)->mods + i);		\
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
#define for_each_snd_mod_in_subpath(subpath, i, mod)	\
	for ((i) = 0;					\
	     ((i) < (subpath)->num_mods) &&		\
		     ((mod) = (subpath)->mods[i]);	\
	     (i)++)
#define for_each_snd_mod_in_path(path, i, mod)			\
	for ((i) = 0;						\
	     ((i) < (path)->current_subpath->num_mods) &&	\
		     ((mod) = (path)->current_subpath->mods[i]);\
	     (i)++)

#define snd_soc_mod_id(mod) ((mod) - (mod)->ctr->mods)

static char *snd_soc_mod_name(struct snd_soc_mod *mod)
{
	return mod->name;
}

static int snd_soc_mod_get_rate_via_subpath(struct snd_soc_mod_subpath *subpath, struct snd_soc_mod *mod)
{
	return mod->ctr->get_rate(subpath, mod);
}

static int snd_soc_mod_get_rate_via_path(struct snd_soc_mod_path *path, struct snd_soc_mod *mod)
{
	return snd_soc_mod_get_rate_via_subpath(path->current_subpath, mod);
}

#define snd_soc_mod_get_rate(path, mod) _Generic((path),	  \
struct snd_soc_mod_subpath * :	snd_soc_mod_get_rate_via_subpath, \
struct snd_soc_mod_path * :	snd_soc_mod_get_rate_via_path)(path, mod)

static struct snd_soc_mod *snd_soc_mod_get_prev_mod(struct snd_soc_mod_subpath *subpath, struct snd_soc_mod *mod)
{
	struct snd_soc_mod *m;
	int i;

	for_each_snd_mod_in_subpath(subpath, i, m) {
		if (m == mod) {
			if (i == 0)
				return NULL;
			else
				return subpath->mods[i - 1];
		}
	}

	return NULL;
}

static int snd_soc_mod_common_get_rate(struct snd_soc_mod_subpath *path, struct snd_soc_mod *mod)
{
	struct snd_soc_mod *prev_mod = snd_soc_mod_get_prev_mod(path, mod);

	/*
	 * FIXME:
	 *	48000 default
	 */
	if (!prev_mod)
		return 48000;

	/*
	 * same as previous mod's rate
	 */
	return snd_soc_mod_get_rate(path, prev_mod);
}

static int snd_soc_mod_graph_sanity_check(struct snd_soc_mod_graph *graph)
{
	struct snd_soc_mod_ctr *ctr;
	struct snd_soc_mod *mod;
	int i, j;

	for_each_snd_mod_ctrs(graph, j, ctr) {
		for_each_snd_mods(ctr, i, mod) {
			struct snd_soc_mod *in, *out;
			int t, s;

			for_each_snd_mod_in(mod, t, in) {
				for_each_snd_mod_out(in, s, out) {
					if (out == mod)
						goto found_out;
				}
				printf("%s not found\n", snd_soc_mod_name(mod));
				return -EINVAL;
			found_out:
			}

			for_each_snd_mod_out(mod, t, out) {
				for_each_snd_mod_in(out, s, in) {
					if (in == mod)
						goto found_in;
				}
				printf("%s not found\n", snd_soc_mod_name(mod));
				return -EINVAL;
			found_in:
			}
		}
	}
}

static void snd_soc_mod_init(struct snd_soc_mod_ctr *ctr, struct snd_soc_mod *mod, int i)
{
	mod->ctr = ctr;
	snprintf(mod->name, SND_SOC_MOD_NAME_SIZE, "%s%d", ctr->base_name, i);

	/*
	 * SND_SOC_MOD_DEFINE() might use size 0 array
	 * int this case, it has array pointer. set it as NULL here
	 * */
	if (!mod->num_mod_in)
		mod->mod_in = NULL;
	if (!mod->num_mod_out)
		mod->mod_out = NULL;
}

static int snd_soc_mod_ctr_init(struct snd_soc_mod_graph *graph, struct snd_soc_mod_ctr *ctr)
{
	struct snd_soc_mod *mod;
	int i;

	if (!ctr->get_rate)
		return -EINVAL;

	ctr->graph = graph;

	for_each_snd_mods(ctr, i, mod)
		snd_soc_mod_init(ctr, mod, i);

	return 0;
}

int snd_soc_mod_graph_init(struct snd_soc_mod_graph *graph)
{
	struct snd_soc_mod_ctr *ctr;
	struct snd_soc_mod *mod;
	int ret, i;

	for_each_snd_mod_ctrs(graph, i, ctr) {
		ret = snd_soc_mod_ctr_init(graph, ctr);
		if (ret < 0)
			return ret;
	}

	return snd_soc_mod_graph_sanity_check(graph);
}

struct snd_soc_mod_subpath *snd_soc_mod_alloc_subpath(struct snd_soc_mod_graph *graph,
						char **path_name, int num_path_name, void *priv)
{
	struct snd_soc_mod_subpath *path;
	struct snd_soc_mod **mods;
	struct snd_soc_mod *prev;
	int i;

	path = malloc(sizeof(*path));
	mods = malloc(sizeof(*mods) * num_path_name);
	if (!path || !mods)
		goto err;

	path->mods	= mods;
	path->num_mods	= num_path_name;
	path->priv	= priv;

	prev = NULL;
	for (i = 0; i < num_path_name; i++) {
		struct snd_soc_mod_ctr *ctr;
		struct snd_soc_mod *mod, *in, *out;
		int j, t;

		/*
		 * find mod from path name
		 * ex) "src0" => &src[0]
		 */
		for_each_snd_mod_ctrs(graph, j, ctr)
			for_each_snd_mods(ctr, t, mod)
				if (strncmp(path_name[i], snd_soc_mod_name(mod), strlen(path_name[i])) == 0)
					goto found;

		printf("can't find path(%s) in graph(%s)\n", path_name[i], graph->name);
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

		printf("can't find %s in %s's mod_in \n", snd_soc_mod_name(prev), path_name[i]);
		goto free;
	in_check_ok:
		/*
		 * In above sample case, ssi0 (= last mod) has no mod_out
		 */
		if (!mod->num_mod_out && (i + 1) == num_path_name)
			goto out_check_ok;

		for_each_snd_mod_out(prev, t, out)
			if (out == mod)
				goto out_check_ok;
		printf("can't find path(%s) in mod(%s)'s out \n", path_name[i], snd_soc_mod_name(mod));
		goto free;
	out_check_ok:
		prev = mod;
		mods[i] = mod;
	}

	return path;
free:
	free(path);
	free(mods);
err:
	return NULL;
}

struct snd_soc_mod_path *snd_soc_mod_alloc_path(int num_subpath)
{
	struct snd_soc_mod_path *path;
	struct snd_soc_mod_subpath **subpath;
	int i;

	path	= malloc(sizeof(*path));
	subpath	= malloc(sizeof(*subpath) * num_subpath);
	if (!path || !subpath)
		goto err;

	path->subpath		= subpath;
	path->num_subpath	= num_subpath;

	/* remove me */
	for (i = 0; i < num_subpath; i++)
		path->subpath[i] = NULL;

	return path;
err:
	free(path);
	free(subpath);

	return NULL;
}

int snd_soc_mod_path_set(struct snd_soc_mod_path *path, int id, struct snd_soc_mod_subpath *subpath)
{
	if (id < 0 ||
	    id >= path->num_subpath)
		return -EINVAL;
	if (path->subpath[id]) // no overwrite
		return -EINVAL;

	path->subpath[id] = subpath;

	/* default path */
	if (!path->current_subpath)
		path->current_subpath = subpath;

	return 0;
}

int snd_soc_mod_path_switch(struct snd_soc_mod_path *path, int id)
{
	if (id < 0 ||
	    id >= path->num_subpath)
		return -EINVAL;
	if (!path->subpath[id])
		return -EINVAL;

	path->current_subpath = path->subpath[id];

	return 0;
}

#define SND_SOC_MOD_DEFINE(in, out, p)		\
{						\
	.mod_in		= in,			\
	.num_mod_in	= ARRAY_SIZE(in),	\
	.mod_out	= out,			\
	.num_mod_out	= ARRAY_SIZE(out),	\
	.priv		= p,			\
}

#define SND_SOC_MOD_CTR_DEFINE(a, p)	\
	.base_name	= #a,		\
	.mods		= a,		\
	.num_mods	= ARRAY_SIZE(a),\
	.priv		= p

#define SND_SOC_MOD_GRAPH_DEFINE(a, p)\
	.name		= #a,		\
	.ctrs		= a,		\
	.num_ctrs	= ARRAY_SIZE(a),\
	.priv		= p

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

struct snd_soc_src_priv {
	int rate;
} src_priv[] = {
	{.rate = 0,	},	// no rate change
	{.rate = 44100	},	// to 44100
	{.rate = 44100	},	// to 44100
	{.rate = 96000	},	// to 96000
};

static int snd_soc_src_get_rate(struct snd_soc_mod_subpath *path, struct snd_soc_mod *mod)
{
	struct snd_soc_src_priv *priv = snd_soc_mod_to_priv(mod);

	if (priv->rate)
		return priv->rate;

	return snd_soc_mod_common_get_rate(path, mod);
}

struct snd_soc_mod *src_in[] = {};
struct snd_soc_mod *src0_out[] = {&mix[0], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *src1_out[] = {&mix[0], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *src2_out[] = {&mix[1], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *src3_out[] = {&mix[1], &ssi[0], &ssi[1], &ssi[2], &ssi[3] };

struct snd_soc_mod src[] = {
	SND_SOC_MOD_DEFINE(src_in, src0_out, &src_priv[0]),
	SND_SOC_MOD_DEFINE(src_in, src1_out, &src_priv[1]),
	SND_SOC_MOD_DEFINE(src_in, src2_out, &src_priv[2]),
	SND_SOC_MOD_DEFINE(src_in, src3_out, &src_priv[3]),
};

struct snd_soc_mod_ctr src_ctr = {
	SND_SOC_MOD_CTR_DEFINE(src, NULL),
	.get_rate = snd_soc_src_get_rate
};

struct snd_soc_mod *mix0_in[] = {&src[0], &src[1]};
struct snd_soc_mod *mix1_in[] = {&src[2], &src[3]};
struct snd_soc_mod *mix0_out[] = {&dvc[0]};
struct snd_soc_mod *mix1_out[] = {&dvc[1]};

struct snd_soc_mod mix[] = {
	SND_SOC_MOD_DEFINE(mix0_in, mix0_out, NULL),
	SND_SOC_MOD_DEFINE(mix1_in, mix1_out, NULL),
};

struct snd_soc_mod_ctr mix_ctr = {
	SND_SOC_MOD_CTR_DEFINE(mix, NULL),
	.get_rate = snd_soc_mod_common_get_rate
};

struct snd_soc_mod *dvc0_in[] = {&mix[0]};
struct snd_soc_mod *dvc1_in[] = {&mix[1]};
struct snd_soc_mod *dvc0_out[] = {&ssi[0], &ssi[1], &ssi[2], &ssi[3] };
struct snd_soc_mod *dvc1_out[] = {&ssi[0], &ssi[1], &ssi[2], &ssi[3] };

struct snd_soc_mod dvc[] = {
	SND_SOC_MOD_DEFINE(dvc0_in, dvc0_out, NULL),
	SND_SOC_MOD_DEFINE(dvc1_in, dvc1_out, NULL),
};

struct snd_soc_mod_ctr dvc_ctr = {
	SND_SOC_MOD_CTR_DEFINE(dvc, NULL),
	.get_rate = snd_soc_mod_common_get_rate
};

struct snd_soc_mod *ssi0_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};
struct snd_soc_mod *ssi1_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};
struct snd_soc_mod *ssi2_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};
struct snd_soc_mod *ssi3_in[] = {&dvc[0], &dvc[1], &src[0], &src[1], &src[2], &src[3]};

struct snd_soc_mod *ssi_out[] = {};

struct snd_soc_mod ssi[] = {
	SND_SOC_MOD_DEFINE(ssi0_in, ssi_out, NULL),
	SND_SOC_MOD_DEFINE(ssi1_in, ssi_out, NULL),
	SND_SOC_MOD_DEFINE(ssi2_in, ssi_out, NULL),
	SND_SOC_MOD_DEFINE(ssi3_in, ssi_out, NULL),
};

struct snd_soc_mod_ctr ssi_ctr = {
	SND_SOC_MOD_CTR_DEFINE(ssi, NULL),
	.get_rate = snd_soc_mod_common_get_rate
};

struct snd_soc_mod_ctr *my_graph[] = {
	&src_ctr,
	&mix_ctr,
	&dvc_ctr,
	&ssi_ctr,
};

struct snd_soc_mod_graph graph = {
	SND_SOC_MOD_GRAPH_DEFINE(my_graph, NULL),
};

//char *my_path[] = {"src0", "mix1", "dvc0", "ssi3"}; // sanity error
char *my_path0[]   = {"src2", "mix1", "dvc1", "ssi2"};
char *my_path1[]   = {"src3", "mix1", "dvc1", "ssi2"};

void main(void)
{
	struct snd_soc_mod *mod;
	struct snd_soc_mod_subpath *subpath0, *subpath1;
	struct snd_soc_mod_path *path;
	int i, ret;

	ret = snd_soc_mod_graph_init(&graph);
	if (ret < 0)
		return;

	subpath0 = snd_soc_mod_alloc_subpath(&graph, my_path0, ARRAY_SIZE(my_path0), NULL);
	subpath1 = snd_soc_mod_alloc_subpath(&graph, my_path1, ARRAY_SIZE(my_path1), NULL);
	if (!subpath0 || !subpath1)
		goto err;

	for_each_snd_mod_in_subpath(subpath0, i, mod)
		printf("%s (%d), ", snd_soc_mod_name(mod), snd_soc_mod_get_rate(subpath0, mod));
	printf("\n");
	for_each_snd_mod_in_subpath(subpath1, i, mod)
		printf("%s (%d), ", snd_soc_mod_name(mod), snd_soc_mod_get_rate(subpath1, mod));
	printf("\n");

	path = snd_soc_mod_alloc_path(2);
	if (!path)
		goto err;

	ret = snd_soc_mod_path_set(path, 0, subpath0);
	if (ret < 0)
		goto err;

	ret = snd_soc_mod_path_set(path, 1, subpath1);
	if (ret < 0)
		goto err;

	for_each_snd_mod_in_path(path, i, mod)
		printf("%s (%d), ", snd_soc_mod_name(mod), snd_soc_mod_get_rate(path, mod));
	printf("\n");

	ret = snd_soc_mod_path_switch(path, 1);
	if (ret < 0)
		goto err;

	for_each_snd_mod_in_path(path, i, mod)
		printf("%s (%d), ", snd_soc_mod_name(mod), snd_soc_mod_get_rate(path, mod));
	printf("\n");

	return;
err:
	printf("error\n");
	return;
}
