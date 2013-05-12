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

// Standard includes
#include <time.h>
#include <iostream>
#include <string>
#include <vector>

// Boost includes
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>

#ifdef GR_CTRLPORT
#include <rpcregisterhelpers.h>
#endif
#include "receiver.h"


#define FFT_SIZE     4000
#define AUDIO_RATE  96000

static long fft_delay_msec = 10;

/*! \brief FFT thread function.
 *  \param rx The active instance of the receiver object.
 *
 * The FFT thread function rund periodically and performs the following
 * tasks:
 *   - Get new FFT data and scale the FFT properly
 *   - Calculate SNR for both receiver channels
 *   - Send SNR for the active channel to the audio indicator.
 */
static void fft_thread_func(receiver *rx)
{
    for(;;)
    {
        rx->process_fft();
        rx->process_snr();

        try
        {
            // Sleep and check for interrupt.
            // To check for interrupt without sleep, use boost::this_thread::interruption_point()
            // which also throws boost::thread_interrupted
            boost::this_thread::sleep(boost::posix_time::milliseconds(fft_delay_msec));
        }
        catch(boost::thread_interrupted&)
        {
            std::cout << "FFT thread is stopped" << std::endl;
            return;
        }
    }
}

/*! Get current date/time, format is YYYYMMDD-HHMMSS */
const char* currentDateTime() {
    time_t     now = time(0);
    struct tm tstruct;
    static char buf[20];

    tstruct = *gmtime(&now);
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y%m%d-%H%M%S", &tstruct);

    return buf;
}

/*! \brief Public contructor.
 *  \param name The receiver name. Used for ctrlport names.
 *  \param input Input device specifier (see below).
 *  \param output Output file name. Using stdout if empty.
 *  \param quad_rate Quadrature rate in samples per second.
 * 
 * The input can be a complex I/Q file or a USRP device. I/Q file is selected if the device string
 * is of the form "file:/some/path", otherwise UHD is assumed with subdev in the string.
 * 
 * \todo Use gr-osmosdr as soon as it support gnuradio 3.7
 */
receiver::receiver(const std::string name, const std::string input, const std::string output, double quad_rate)
{

    if (name.empty())
        d_name = "strx";
    else
        d_name = name;

    // initialize misc parameters
    d_running = false;
    d_quad_rate = quad_rate;
    d_lnb_lo = 0.0;

    // Initialize DSP blocks
    tb = gr_make_top_block(d_name);

    src = strx::source_c::make(input, d_quad_rate);
    fft = strx::fft_c::make(FFT_SIZE);
    iqrec = blocks::file_sink::make(sizeof(gr_complex), "/tmp/strx.raw");
    iqrec->set_unbuffered(true);
    iqrec->close();
    d_recording = 0;

    // channel filter setup
    d_ch_offs[0] = -1.0e6;
    d_ch_offs[1] = 1.0e6;
    d_ch = 0;
    d_cutoff = 400e3;
    taps = filter::firdes::low_pass(1.0, d_quad_rate, d_cutoff, d_cutoff);
    filter = filter::freq_xlating_fir_filter_ccf::make(2, taps, d_ch_offs[d_ch], d_quad_rate);

    // audio SSI
    trk_sig = analog::sig_source_f::make(AUDIO_RATE, analog::GR_SIN_WAVE, 10, 0.5, 0);
    trk_snd = audio::sink::make(AUDIO_RATE, "pulse", true);

    // other blocks
    demod = analog::quadrature_demod_cf::make(1.f);
    iir = filter::single_pole_iir_filter_ff::make(1.e-3);
    sub = blocks::sub_ff::make();
    clock_recov = digital::clock_recovery_mm_ff::make(8.f, 10.e-3f, 10.e-3f, 1.e-3f, 10.e-3f);


    if (output.empty())
    {
        fifo = blocks::file_sink::make(sizeof(float), "/dev/fd/1");
    }
    else
    {
        fifo = blocks::file_sink::make(sizeof(float), output.c_str());
    }

    // Initialize FFT
    fft_thread = boost::thread(&fft_thread_func, this);
    d_fftAvg = 0.5f;
    d_fftLen = 0;
    d_fftData = new std::complex<float>[MAX_FFT_SIZE];
    d_realFftData = new float[MAX_FFT_SIZE];
    d_iirFftData = new float[MAX_FFT_SIZE];
    for (int i = 0; i < MAX_FFT_SIZE; i++)
        d_iirFftData[i] = -120.0f;  // dBFS

    // initialize SNR
    d_signal = -120.0;
    d_noise  = -120.0;
    d_snr_alpha = 0.1;
    d_snr_alpha_inv = 1.0 - d_snr_alpha;
    d_last_snr = 0.0;

    // initialize control port
#ifdef GR_CTRLPORT
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, double>
            (
                d_name,   // const std::string& name,
                "frequency",  // const char* functionbase,
                this,      // T* obj,
                &receiver::rx_freq, // Tfrom (T::*function)(),
                pmt::mp(50.0e6), pmt::mp(2.0e9), pmt::mp(100.0e6),
                "Hz", // const char* units_ = "",
                "Receiver frequency (read only)", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

    // FFT
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, std::vector<float> >
            (
                d_name,   // const std::string& name,
                "fft",  // const char* functionbase,
                this,      // T* obj,
                &receiver::get_fft_data, // Tfrom (T::*function)(),
                pmt::make_f32vector(0, -200.0f),
                pmt::make_f32vector(0, 0.0f),
                pmt::make_f32vector(0, -120.0f),
                "dBFS", // const char* units_ = "",
                "Baseband FFT", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPXY | DISPOPTSCATTER
            )
    ));

    // SNR
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, double>
            (
                d_name,   // const std::string& name,
                "snn",  // const char* functionbase,
                this,      // T* obj,
                &receiver::get_snr, // Tfrom (T::*function)(),
                pmt::mp(0.0),
                pmt::mp(100.0),
                pmt::mp(0.0),
                "dB", // const char* units_ = "",
                "Signal+noise to noise ratio", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPXY | DISPOPTSCATTER
            )
    ));

    // Filter offset (aka. receiver LO)
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, double>
            (
                d_name,   // const std::string& name,
                "offset",  // const char* functionbase,
                this,      // T* obj,
                &receiver::get_filter_offset, // Tfrom (T::*function)(),
                pmt::mp(-d_quad_rate/2.0), pmt::mp(d_quad_rate/2.0), pmt::mp(0.0),
                "Hz", // const char* units_ = "",
                "Channel offset", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_set<receiver, double>
            (
                d_name,   // const std::string& name,
                "offset",  // const char* functionbase,
                this,      // T* obj,
                &receiver::set_filter_offset, // Tfrom (T::*function)(),
                pmt::mp(-d_quad_rate/2.0), pmt::mp(d_quad_rate/2.0), pmt::mp(0.0),
                "Hz", // const char* units_ = "",
                "Channel offset", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

    // Filter cutoff
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, double>
            (
                d_name,   // const std::string& name,
                "cutoff",  // const char* functionbase,
                this,      // T* obj,
                &receiver::get_filter_cutoff, // Tfrom (T::*function)(),
                pmt::mp(-d_quad_rate/2.0), pmt::mp(d_quad_rate/2.0), pmt::mp(d_cutoff),
                "Hz", // const char* units_ = "",
                "Filter cutoff", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_set<receiver, double>
            (
                d_name,   // const std::string& name,
                "cutoff",  // const char* functionbase,
                this,      // T* obj,
                &receiver::set_filter_cutoff, // Tfrom (T::*function)(),
                pmt::mp(-d_quad_rate/2.0), pmt::mp(d_quad_rate/2.0), pmt::mp(d_cutoff),
                "Hz", // const char* units_ = "",
                "Filter cutoff", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

    // Channel selector
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, int>
            (
                d_name,   // const std::string& name,
                "channel",  // const char* functionbase,
                this,      // T* obj,
                &receiver::get_active_channel, // Tfrom (T::*function)(),
                pmt::mp(0), pmt::mp(MAX_CHAN), pmt::mp(d_ch),
                "", // const char* units_ = "",
                "Active channel", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_set<receiver, int>
            (
                d_name,   // const std::string& name,
                "channel",  // const char* functionbase,
                this,      // T* obj,
                &receiver::set_active_channel, // Tfrom (T::*function)(),
                pmt::mp(0), pmt::mp(MAX_CHAN), pmt::mp(d_ch),
                "", // const char* units_ = "",
                "Active channel", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

    // I/Q recording
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_get<receiver, int>
            (
                d_name,   // const std::string& name,
                "iqrec",  // const char* functionbase,
                this,      // T* obj,
                &receiver::iqrec_enabled, // Tfrom (T::*function)(),
                pmt::mp(0), pmt::mp(1), pmt::mp(0),
                "ena", // const char* units_ = "",
                "I/Q recording", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));
    add_rpc_variable(rpcbasic_sptr(new rpcbasic_register_set<receiver, int>
            (
                d_name,   // const std::string& name,
                "iqrec",  // const char* functionbase,
                this,      // T* obj,
                &receiver::iqrec_enable, // Tfrom (T::*function)(),
                pmt::mp(0), pmt::mp(1), pmt::mp(0),
                "ena", // const char* units_ = "",
                "I/Q recording", // const char* desc_ = "",
                RPC_PRIVLVL_MIN,
                DISPNULL
            )
    ));

#endif

    connect_all();
}

/*! \brief Public destructor. */
receiver::~receiver()
{
    fft_thread.interrupt();
    fft_thread.join();
    tb->stop();

    delete [] d_fftData;
    delete [] d_realFftData;
    delete [] d_iirFftData;
}

/*! \brief Start the receiver. */
void receiver::start()
{
    if (!d_running)
    {
        tb->run();
        //tb->start();
        d_running = true;
    }
}

/*! \brief Stop the receiver. */
void receiver::stop()
{
    if (d_running)
    {
        tb->stop();
        tb->wait(); // If the graph is needed to run again, wait() must be called after stop
        d_running = false;
    }
}

void receiver::set_antenna(std::string antenna)
{
    src->set_antenna(antenna);
}

/*! Set new RF frequency.
 * \param freq_hz The new frequency in Hz.
 */
void receiver::set_rf_freq(double freq_hz)
{
    src->set_freq(freq_hz);
}

/*! Get current RF frequency.
 * \returns The current RF frequency or 0 if using a file input.
 */
double receiver::rf_freq(void)
{
    return src->get_freq();
}

/*! Get receiver freuqency.
 *
 * The receiver frequency is the RF frequency + the LN LO.
 */
double receiver::rx_freq(void)
{
    return (rf_freq() + d_lnb_lo);
}

/*! Get RF frequency range for the current device
 * \param[out] start The lower end of the frequency range.
 * \param[out] stop The upper end of the frequency range.
 * \param[out] step Not used
 */
void receiver::rf_freq_range(double *start, double *stop, double *step)
{
    src->get_freq_range(start, stop, step);
}

/*! Set new RF gain.
 * \param gain The new gain in dB.
 */
void receiver::set_rf_gain(double gain)
{
    src->set_gain(gain);
}

/*! Get current RF gain.
 * \returns The current RF gain.
 */
double receiver::rf_gain(void)
{
    return src->get_gain();
}

/*! Get gain range for current USRP device.
 * \param[out] start The lower limit of the gain range.
 * \param[out] stop The upper limit of the gain range.
 * \param[out] step Not used
 */
void receiver::rf_gain_range(double *start, double *stop, double *step)
{
    src->get_gain_range(start, stop, step);
}

/*! Set channel filter offset (aka. receiver LO).
 * \param freq_hz The new offset frequency in Hz.
 */
void receiver::set_filter_offset(double freq_hz)
{
    filter->set_center_freq(freq_hz);
    d_ch_offs[d_ch] = freq_hz;
}

/*! Get channel filter offset (aka. receiver LO).
 * \returns The current channel filter offset frequency in Hz.
 */
double receiver::get_filter_offset(void)
{
    return filter->center_freq();
}

/*! Set new filter cutoff and transition width
 *  \param freq_hz The new cutoff frequency.
 *
 * This function generates and sets new filter taps for the frequency
 * translating FIR filter (our channel filter). The transition with is set
 * to be equal to the cutoff frequency.
 */
void receiver::set_filter_cutoff(double freq_hz)
{
    if (freq_hz < 50e3)
        return;

    d_cutoff = freq_hz;
    taps = filter::firdes::low_pass(1.0, d_quad_rate, d_cutoff, d_cutoff);
    filter->set_taps(taps);
}

/*! Get current filter cutoff (1/2 width) */
double receiver::get_filter_cutoff(void)
{
    return d_cutoff;
}

/*! Select new channel */
void receiver::set_active_channel(int channel)
{
    if (channel <= MAX_CHAN)
    {
        d_ch = channel;
        filter->set_center_freq(d_ch_offs[d_ch]);
    }
}

/*! Get active channel */
int  receiver::get_active_channel(void)
{
    return d_ch;
}


/*! Set FFT rate.
 *  \param rate The new FFT rate in Hz (updates per second).
 *
 * The FFT rate is the frequency by which the FFT thread is running.
 *
 * \sa fft_thread_func
 */
void receiver::set_fft_rate(long rate)
{
    fft_delay_msec = 1000 / rate;
}

/*! Get latest FFT data.
 *  \return A vector of floats containing the latest FFT data (dBFS units).
 *
 * This function is used by the control port to fetch the latest FFT data.
 * The FFT data has already been prostprocessed and converted to dBFS, thus
 * all we need to do here is to copy the data to a vector.
 */
std::vector<float> receiver::get_fft_data(void)
{
    if (d_iirFftData != NULL && d_fftLen > 0)
    {
        fft_lock.lock();
        std::vector<float> vec(d_iirFftData, d_iirFftData+d_fftLen);
        fft_lock.unlock();
        return vec;
    }
    else
    {
        std::vector<float> vec;
        return vec;
    }
}

double receiver::get_snr(void)
{
    return d_last_snr;
}

/*! Connect all blocks in the receiver chain. */
void receiver::connect_all()
{
    tb->connect(src, 0, filter, 0);
    tb->connect(src, 0, fft, 0);
    tb->connect(src, 0, iqrec, 0);
    tb->connect(filter, 0, demod, 0);
    tb->connect(demod, 0, iir, 0);
    tb->connect(demod, 0, sub, 0);
    tb->connect(iir, 0, sub, 1);
    tb->connect(sub, 0, clock_recov, 0);
    tb->connect(clock_recov, 0, fifo, 0);

    tb->connect(trk_sig, 0, trk_snd, 0);
}

/*! \brief Process FFT data.
 *
 * Perform FFT data processing consisting of the following steps:
 * - Fetch the latest FFT data
 * - Scale it according to FFT size
 * - Convert to dBFS
 * - Calculate the average if averaging is enabled
 * The processing is performed at each cycle even though we could postpone the
 * conversion to dBFS and averaging to only happen when someone asks for it. I
 * expect though that this extra processing will reduce the control port latency
 * since the requested data is already avaialble.
 */
void receiver::process_fft(void)
{
    int i;
    float gain;
    float pwr;
    std::complex<float> pt;             /* a single FFT point used in calculations */
    std::complex<float> scaleFactor;    /* normalizing factor (fftsize cast to complex) */

    fft->get_fft_data(d_fftData, d_fftLen);
    if (d_fftLen == 0)
        return;

    fft_lock.lock();
    scaleFactor = std::complex<float>((float)d_fftLen);

    // Normalize, calculcate power and shift the FFT
    for (i = 0; i < d_fftLen; i++)
    {
        // normalize and shift
        if (i < d_fftLen/2)
        {
            pt = d_fftData[d_fftLen/2+i] / scaleFactor;
        }
        else
        {
            pt = d_fftData[i-d_fftLen/2] / scaleFactor;
        }
        pwr = pt.imag()*pt.imag() + pt.real()*pt.real();

        /* calculate power in dBFS */
        d_realFftData[i] = 10.0 * log10(pwr + 1.0e-20);

        /* FFT averaging (aka. video filter) */
        gain = d_fftAvg * (150.0+d_realFftData[i])/150.0;

        d_iirFftData[i] = (1.0 - gain) * d_iirFftData[i] + gain * d_realFftData[i];
    }

    fft_lock.unlock();
}

/*! \brief Calculate signal to noise ratios. */
void receiver::process_snr(void)
{
    double rbw = d_quad_rate / FFT_SIZE;   // FFT resolution bandwidth
    int numbins = 200e3 / (int)rbw;
    int startbin;
    int i;
    double sum = 0.;
    double noise, signal;

    if (d_fftLen <= 0)
        return;

    fft_lock.lock();

    // average noise power calculated around 0 Hz
    startbin = d_fftLen/2 - numbins/2;
    for (i = startbin; i < startbin + numbins; i++)
    {
        sum += (double)d_realFftData[i];
    }
    d_noise = sum / (double) numbins;

    // average signal power calculated around offset
    sum = 0.;
    startbin += (int)(d_ch_offs[d_ch] / rbw) - numbins/2;
    for (i = startbin; i < startbin + numbins; i++)
    {
        sum += (double)d_realFftData[i];
    }
    fft_lock.unlock();

    d_signal = sum / (double) numbins;

    double this_snr = d_signal - d_noise;
    if (this_snr < 0.0)
        this_snr = 0.0;

    // single pole iir
    d_last_snr *= d_snr_alpha_inv;
    d_last_snr += d_snr_alpha * this_snr;

    trk_sig->set_frequency(snr_to_freq(d_last_snr));
}

/*! Convert SNR to audio frequency. */
double receiver::snr_to_freq(double snr)
{
#define SNR_MIN  5.0
#define SNR_MAX 30.0
#define F_MIN   300.0
#define F_MAX   1.e3
#define SLOPE (F_MAX-F_MIN)/(SNR_MAX-SNR_MIN)

    double freq_out;

    if (snr > SNR_MAX)
        snr = SNR_MAX;
    else if (snr < SNR_MIN)
        snr = SNR_MIN;

    freq_out = F_MIN + SLOPE * (snr-SNR_MIN);

    std::cout << snr << " dB => " << freq_out << std::endl;

    return freq_out;
}

/*! \brief Enable or disable I/Q recording.
 *  \param enable Whether recording should be enabled or disabled.
 *
 * When recording is enabled we start recording an I/Q file connetcted
 * directly to to the UHD source. The filename is of the form:
 *   sapphire_freq_rate_YYYYMMDD-HHMMSS.raw
 * This function can also be used to restart a recording into a new file.
 */
void receiver::iqrec_enable(int enable)
{
    if (d_recording)
    {
        // stop ongoing recording
        iqrec->close();
    }

    if (enable)
    {
        // start new recording
        int freq = (int)(rf_freq() / 1.e3);   // frequency in kHz
        int rate = (int)(d_quad_rate / 1.e6); // sample rate in Msps
        char buff[80];

        sprintf(buff, "sapphire_%dkHz_%dMsps_%s.raw", freq, rate, currentDateTime());

        iqrec->open(buff);
        iqrec->do_update();
    }

    d_recording = enable;
}

int receiver::iqrec_enabled(void)
{
    return d_recording;
}
