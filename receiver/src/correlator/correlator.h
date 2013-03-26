#ifndef INCLUDED_CORRELATOR_H
#define INCLUDED_CORRELATOR_H

#include <gr_block.h>
#include "api.h"

namespace strx {

    class CORRELATOR_API correlator : virtual public gr_block
    {
    public:

        // strx::correlator::sptr
        typedef boost::shared_ptr<correlator> sptr;

        static sptr make();
    };

} /* namespace strx */

#endif /* INCLUDED_CORRELATOR_H */
