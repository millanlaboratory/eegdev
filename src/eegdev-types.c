/*
	Copyright (C) 2010  EPFL (Ecole Polytechnique Fédérale de Lausanne)
	Nicolas Bourdaud <nicolas.bourdaud@epfl.ch>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#if HAVE_CONFIG_H
# include <config.h>
#endif
#include <string.h>
#include "eegdev-types.h"

// Prototype of a generic type scale and cast function
#define DEFINE_CAST_FN(fnname, tsrc, tdst, uniontype)			\
static void fnname(void* restrict d, const void* restrict s, union gval sc, size_t len)	\
{									\
	const tsrc* src = s;						\
	tdst* dst = d;							\
	tdst scale = sc.uniontype;					\
	while(len) {							\
		*dst = scale * ((tdst)(*src));				\
		src++;							\
		dst++;							\
		len -= sizeof(*src);					\
	}								\
}						

// Prototype of a generic type cast function
#define DEFINE_CASTNOSC_FN(fnname, tsrc, tdst)				\
static void fnname(void* restrict d, const void* restrict s, union gval sc, size_t len)	\
{									\
	(void)sc;							\
	const tsrc* src = s;						\
	tdst* dst = d;							\
	while(len) {							\
		*dst = ((tdst)(*src));					\
		src++;							\
		dst++;							\
		len -= sizeof(*src);					\
	}								\
}						

static void identity(void* restrict d, const void* restrict s, union gval sc, size_t len)
{
	(void)sc;
	memcpy(d, s, len);
}

// Declaration/definition of type cast and scale functions
DEFINE_CAST_FN(cast_i32_i32, int32_t, int32_t, i32val)
DEFINE_CAST_FN(cast_i32_d, int32_t, double, dval)
DEFINE_CAST_FN(cast_d_i32, double, int32_t, i32val)
DEFINE_CAST_FN(cast_i32_f, int32_t, float, fval)
DEFINE_CAST_FN(cast_f_i32, float, int32_t, i32val)
DEFINE_CAST_FN(cast_f_d, float, double, dval)
DEFINE_CAST_FN(cast_d_f, double, float, fval)
DEFINE_CAST_FN(cast_f_f, float, float, fval)
DEFINE_CAST_FN(cast_d_d, double, double, dval)

// Declaration/definition of type cast functions
DEFINE_CASTNOSC_FN(castnosc_i32_d, int32_t, double)
DEFINE_CASTNOSC_FN(castnosc_d_i32, double, int32_t)
DEFINE_CASTNOSC_FN(castnosc_i32_f, int32_t, float)
DEFINE_CASTNOSC_FN(castnosc_f_i32, float, int32_t)
DEFINE_CASTNOSC_FN(castnosc_f_d, float, double)
DEFINE_CASTNOSC_FN(castnosc_d_f, double, float)

static cast_function convtable[3][2][3] = {
	[EGD_INT32] = {
		[0] = {[EGD_INT32] = identity,
		       [EGD_FLOAT] = castnosc_i32_f,
		       [EGD_DOUBLE] = castnosc_i32_d},
		[1] = {[EGD_INT32] = cast_i32_i32, 
		       [EGD_FLOAT] = cast_i32_f,
		       [EGD_DOUBLE] = cast_i32_d},
	},
	[EGD_FLOAT] = {
		[0] = {[EGD_INT32] = castnosc_f_i32,
		       [EGD_FLOAT] = identity,
		       [EGD_DOUBLE] = castnosc_f_d},
		[1] = {[EGD_INT32] = cast_f_i32,
		       [EGD_FLOAT] = cast_f_f,
		       [EGD_DOUBLE] = cast_f_d},
	},
	[EGD_DOUBLE] = {
		[0] = {[EGD_INT32] = castnosc_d_i32,
		       [EGD_FLOAT] = castnosc_d_f,
		       [EGD_DOUBLE] = identity},
		[1] = {[EGD_INT32] = cast_d_i32,
		       [EGD_FLOAT] = cast_d_f,
		       [EGD_DOUBLE] = cast_d_d},
	}
};


LOCAL_FN
unsigned int egd_get_data_size(unsigned int type)
{
	unsigned int size = 0;

	if (type == EGD_INT32)		
		size = sizeof(int32_t);
	else if (type == EGD_FLOAT)
		size = sizeof(float);
	else if (type == EGD_DOUBLE)
		size = sizeof(double);
	
	return size;
}

LOCAL_FN
cast_function egd_get_cast_fn(unsigned int itype, unsigned int otype, unsigned int scaling)
{
	if ((itype >= EGD_NUM_DTYPE) || (otype >= EGD_NUM_DTYPE))
		return NULL;

	scaling = scaling ? 1 : 0;

	return convtable[itype][scaling][otype];
}
