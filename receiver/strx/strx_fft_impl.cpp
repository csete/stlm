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
#include <gnuradio/io_signature.h>
#include "strx_fft_impl.h"

namespace strx {

    fft_c::sptr fft_c::make(int fftsize, int wintype)
    {
        return gnuradio::get_initial_sptr(new fft_c_impl(fftsize, wintype));
    }

    fft_c_impl::fft_c_impl(int fftsize, int wintype)
      : gr::sync_block("strx_fft_c",
                       gr::io_signature::make(1, 1, sizeof (gr_complex)),
                       gr::io_signature::make(0, 0, 0)),
        d_fftsize(fftsize),
        d_wintype(-1)
    {
        // create FFT object
        d_fft = new gr::fft::fft_complex(d_fftsize, true, 1);

        // allocate circular buffer
        d_cbuf.set_capacity(d_fftsize);

        // create FFT window
        set_window_type(wintype);
    }

    fft_c_impl::~fft_c_impl()
    {
        delete d_fft;
    }

    /*! \brief Receiver FFT work method.
     *  \param noutput_items
     *  \param input_items
     *  \param output_items
     *
     * This method does nothing except throwing the incoming samples into the
     * circular buffer. FFT is only executed when the GUI asks for new FFT data via get_fft_data().
     */
    int fft_c_impl::work(int noutput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items)
    {
        int i;
        const gr_complex *in = (const gr_complex*)input_items[0];
        (void) output_items;

        // just throw new samples into the buffer
        boost::mutex::scoped_lock lock(d_mutex);
        for (i = 0; i < noutput_items; i++)
        {
            d_cbuf.push_back(in[i]);
        }

        return noutput_items;
    }

    void fft_c_impl::get_fft_data(std::complex<float>* fftPoints, int &fftSize)
    {
        boost::mutex::scoped_lock lock(d_mutex);

        if (d_cbuf.size() < d_fftsize)
        {
            // not enough samples in the buffer
            fftSize = 0;

            return;
        }

        // perform FFT
        do_fft(d_cbuf.linearize(), d_cbuf.size());  // FIXME: array_one() and two() may be faster
        d_cbuf.clear();

        // get FFT data
        memcpy(fftPoints, d_fft->get_outbuf(), sizeof(gr_complex)*d_fftsize);
        fftSize = d_fftsize;
    }

    /*! \brief Compute FFT on the available input data.
     *  \param data_in The data to compute FFT on.
     *  \param size The size of data_in.
     *
     * Note that this function does not lock the mutex since the caller, get_fft_data()
     * has alrady locked it.
     */
    void fft_c_impl::do_fft(const gr_complex *data_in, int size)
    {
        // apply window, then execute FFT
        if (d_window.size())
        {
            gr_complex *dst = d_fft->get_inbuf();
            for (int i = 0; i < size; i++)
                dst[i] = data_in[i] * d_window[i];
        }
        else
        {
            memcpy(d_fft->get_inbuf(), data_in, sizeof(gr_complex)*size);
        }

        d_fft->execute();
    }

    void fft_c_impl::set_fft_size(int fftsize)
    {
        if (fftsize != d_fftsize)
        {
            boost::mutex::scoped_lock lock(d_mutex);

            d_fftsize = fftsize;

            // clear and resize circular buffer
            d_cbuf.clear();
            d_cbuf.set_capacity(d_fftsize);

            // reset window
            d_wintype = -1;
            set_window_type(d_wintype);

            // reset FFT object (also reset FFTW plan)
            delete d_fft;
            d_fft = new gr::fft::fft_complex(d_fftsize, true);
        }
    }

    int fft_c_impl::get_fft_size()
    {
        return d_fftsize;
    }

    void fft_c_impl::set_window_type(int wintype)
    {
        if (wintype == d_wintype)
        {
            // nothing to do
            return;
        }

        d_wintype = wintype;

        if ((d_wintype < gr::filter::firdes::WIN_HAMMING) || (d_wintype > gr::filter::firdes::WIN_BLACKMAN_hARRIS))
        {
            d_wintype = gr::filter::firdes::WIN_HAMMING;
        }

        d_window.clear();
        d_window = gr::filter::firdes::window((gr::filter::firdes::win_type)d_wintype, d_fftsize, 6.76);
    }

    int fft_c_impl::get_window_type()
    {
        return d_wintype;
    }

} // namespace strx
