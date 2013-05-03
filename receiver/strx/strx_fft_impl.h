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
#ifndef INCLUDED_STRX_FFT_IMPL_H
#define INCLUDED_STRX_FFT_IMPL_H

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <config.h>
#include <fft/fft.h>
#include <filter/firdes.h>
#include <gr_complex.h>

#include "strx_fft.h"

namespace strx {

    class fft_c_impl : public fft_c
    {
    public:
        fft_c_impl(int fftsize=4000, int wintype=gr::filter::firdes::WIN_HAMMING);
        ~fft_c_impl();

        int work(int noutput_items,
                 gr_vector_const_void_star &input_items,
                 gr_vector_void_star &output_items);

        // Public API functions documented in strx_fft.h
        void get_fft_data(std::complex<float>* fft_points, int &fft_size);
        void set_window_type(int wintype);
        int  get_window_type();
        void set_fft_size(int fftsize);
        int get_fft_size();

    private:
        int           d_fftsize;   /*! Current FFT size. */
        int           d_wintype;   /*! Current window type. */
        boost::mutex  d_mutex;     /*! Used to lock FFT output buffer. */
        gr::fft::fft_complex *d_fft;    /*! FFT object. */
        std::vector<float>   d_window; /*! FFT window taps. */
        boost::circular_buffer<gr_complex> d_cbuf; /*! buffer to accumulate samples. */

        void do_fft(const gr_complex *data_in, int size);
    };

} // namespace strx

#endif /* INCLUDED_STRX_SOURCE_C_IMPL_H */
