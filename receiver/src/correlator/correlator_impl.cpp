#include <gr_io_signature.h>
#include <unistd.h>

#include "correlator_impl.h"

namespace strx {

    correlator::sptr correlator::make()
    {
        return gnuradio::get_initial_sptr(new correlator_impl());
    }

    correlator_impl::correlator_impl()
      : gr_block("correlator",
                 gr_make_io_signature(1, 1, sizeof(float)),
                 gr_make_io_signature(1, 1, sizeof(uint8_t)))
    {

    }

    correlator_impl::~correlator_impl()
    {

    }

    void correlator_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
    {
        ninput_items_required[0] = 8*noutput_items + 5;
    }

    int correlator_impl::general_work(int noutput_items,
                                      gr_vector_int &ninput_items,
                                      gr_vector_const_void_star &input_items,
                                      gr_vector_void_star &output_items)
    {
        //const float   *in  = reinterpret_cast<const float *>(input_items[0]);
        //uint8_t       *out = reinterpret_cast<uint8_t *>(output_items[0]);

        // Perform work; read from in, write to out.

        // consume the inputs
        this->consume(0, noutput_items);
        //this->consume_each(noutput_items);

        return noutput_items;
    }

} /* namespace strx */
