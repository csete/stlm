#ifndef INCLUDED_CORRELATOR_IMPL_H
#define INCLUDED_CORRELATOR_IMPL_H

#include "correlator.h"

namespace strx {

    class CORRELATOR_API correlator_impl : public correlator
    {
    private:

    public:
        correlator_impl();

        ~correlator_impl();

        void forecast(int noutput_items, gr_vector_int &ninput_items_required);

        int general_work(int noutput_items,
                         gr_vector_int &ninput_items,
                         gr_vector_const_void_star &input_items,
                         gr_vector_void_star &output_items);
    };

} /* namespace strx */

#endif /* INCLUDED_CORRELATOR_IMPL_H */
