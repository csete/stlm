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
#ifndef INCLUDED_STRX_SOURCE_C_IMPL_H
#define INCLUDED_STRX_SOURCE_C_IMPL_H

#include <blocks/file_source.h>
#include <blocks/throttle.h>
#include <uhd/usrp_source.h>

#include "strx_source_c.h"

class STRX_API strx_source_c_impl : public strx_source_c
{
public:
    /* Public API functions documented in strx_source_c.h */
    void get_freq_range(double *start, double *stop, double *step);
    void set_freq(double freq);
    double get_freq(void);
  
    void set_gain(double gain);
    double get_gain(void);
    void   get_gain_range(double *start, double *stop, double *step);
    
    void set_antenna(std::string antenna);

private:
    strx_source_c_impl(const std::string input = "", double quad_rate=2.0e6);
    friend STRX_API strx_source_c_sptr strx_make_source_c(const std::string input, double quad_rate);

    /*! Input type. */
    enum input_type_e
    {
        INPUT_TYPE_FILE = 0, /*!< Input is complex 2 Msps I/Q file. */
        INPUT_TYPE_UHD  = 1, /*!< Input is a USRP device. */
    };

    input_type_e input_type;

    gr::uhd::usrp_source::sptr                  usrp_src;  /*!< USRP source. */
    gr::blocks::file_source::sptr               file_src;  /*!< I/Q file source. */
    gr::blocks::throttle::sptr                  throttle;  /*!< Rate limiter for file sources. */

    double d_quad_rate; /*!< Quadrature rate. */
    double d_freq;      /*!< Current RF frequency. */
    double d_gain;      /*!< Current gain. */
};

#endif /* INCLUDED_STRX_SOURCE_C_IMPL_H */
