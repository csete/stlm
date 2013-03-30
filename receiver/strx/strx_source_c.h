/* -*- c++ -*- */
/*
 * Copyright 2013 Alexandru Csete, OZ9AEC
 *
 * Strx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Strx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_STRX_SOURCE_C_H
#define INCLUDED_STRX_SOURCE_C_H

#include <gr_hier_block2.h>
#include "strx_api.h"

class strx_source_c;

typedef boost::shared_ptr<strx_source_c> strx_source_c_sptr;

/*! Return a shared_ptr to a new instance of strx_source_c. */
STRX_API strx_source_c_sptr strx_make_source_c(const std::string input="", double quad_rate=2.0e6);

/*! Strx source block. */
class STRX_API strx_source_c : virtual public gr_hier_block2
{
public:
    virtual void   set_freq(double freq) = 0;
    virtual double get_freq() = 0;
    virtual void   get_freq_range(double *start, double *stop, double *step) = 0;

    virtual void   set_gain(double gain) = 0;
    virtual double get_gain() = 0;
    virtual void   get_gain_range(double *start, double *stop, double *step) = 0;
    
    virtual void set_antenna(std::string antenna) = 0;
};

#endif /* INCLUDED_STRX_SOURCE_C_H */
