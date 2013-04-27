/* -*- c++ -*- */
/*
 * Copyright 2013 Alexandru Csete OZ9AEC.
 *
 * Gqrx is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * Gqrx is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Gqrx; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef STRX_FFT_H
#define STRX_FFT_H

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <filter/firdes.h>       /* contains enum win_type */
#include <gr_complex.h>
#include <gr_sync_block.h>

#include "strx_api.h"

#define MAX_FFT_SIZE 32768


namespace strx {

    /*! Strx FFT block.
     * 
     * Compute complex FFT of the received samples.
     *
     * The samples are collected in a circular buffer with size d_fftsize.
     * When users ask for a new set of FFT data via get_fft_data() an FFT is
     * performed on the data stored in the circular buffer provided that the
     * at least d_fftsize samples in the buffer.
     *
     * \note Used qtgui_sink_c as starting point.
     */
    class STRX_API fft_c : virtual public gr_sync_block
    {
    public:

        typedef boost::shared_ptr<fft_c> sptr;

        /*! \brief Return a shared_ptr to a new instance of strx::fft_c.
         *  \param fftsize The FFT size.
         *  \param wintype The window type.
         */
        static sptr make(unsigned int fftsize=4000, int wintype=gr::filter::firdes::WIN_HAMMING);

        /*! \brief Get new FFT data.
         *  \param fft_points The calculated FFT points
         *  \param fft_size The number of points in the fft_points array. This number will either be
         *                   FFT size or 0 if there weren't enough samples accumulated in the buffer.
         */
        virtual void get_fft_data(std::complex<float>* fft_points, unsigned int &fft_size) = 0;

        /*! \brief Set new window type.
         *  \param wintype See filter/firdes.h
         */
        virtual void set_window_type(int wintype) = 0;

        /*! \brief Get current window type.
         *  \returns The current window type, see gnuradio/filter/firdes.h
         */
        virtual int  get_window_type() = 0;

        /*! \brief Set new FFT size.
         *  \param fftsize The new FFT size.
         */
        virtual void set_fft_size(unsigned int fftsize) = 0;

        /*! \brief Get current FFT size.
         *  \returns The current FFT size.
         */
        virtual unsigned int get_fft_size() = 0;

    };

} // namespace strx

#endif // STRX_FFT_H
