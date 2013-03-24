
#include <stdint.h>

/** @brief  KISS protocol special bytes. */
#define KISS_FEND   (0xC0)
#define KISS_FESC   (0xDB)
#define KISS_TFEND  (0xDC)
#define KISS_TFESC  (0xDD)


/** @brief Static memory for the packet transmitter services. */
static unsigned char    *tx_buffer;
static unsigned char    tx_length;
static enum {
    TX_IDLE,
    TX_STARTING,
    TX_IN_PROGRESS,
    TX_DO_ESCAPE
} tx_state = TX_IDLE;


/** @brief Invoked by a TX-ready interrupt service.
 * @return: the next char to transmit.
 */
static unsigned char tx_get_next_char (void) {
    switch (tx_state) {
        case TX_IDLE:
            return KISS_FEND;                   /* In idle mode return a FEND char. */

        case TX_STARTING:
            tx_state = TX_IN_PROGRESS;          /* Switch to IN_PROGRESS. */
            return KISS_FEND;                   /* and return the starting FEND flag. */

        case TX_IN_PROGRESS:
            if (tx_length == 0) {               /* No more data in the buffer. Wrap up with a closing FEND. */
                tx_state == TX_IDLE;            /* Switch state to IDLE. */
                return KISS_FEND;               /* Return the closing flag. */
            }

            /* Transmit the next char in the buffer. */
            switch (*tx_buffer) {
                case KISS_FEND:                 /* Have to transmit a protected FEND char. */
                case KISS_FESC:                 /* Have to transmit a protected FESC char. */
                    tx_state = TX_DO_ESCAPE;    /* Change state to ESCAPE. */
                    return KISS_FESC;           /* and return a FESC char. */

                default:
                    tx_length--;                /* Decrement the packet length counter. */
                    return *(tx_buffer++);      /* Return the char and increment the pointer. */
            }

        case TX_DO_ESCAPE:
            if (*tx_buffer == KISS_FEND) {      /* The escaped char was a FEND. */
                tx_length--;                    /* Decrement the buffer length counter. */
                tx_buffer++;                    /* Step past the now transmitted char. */
                tx_state = TX_IN_PROGRESS;      /* Change state back to IN_PROGRESS. */
                return KISS_TFEND;              /* Return the transposed char. */
            } else {                            /* The escaped char have to have been a FESC. */
                tx_length--;                    /* Decrement the buffer length counter. */
                tx_buffer++;                    /* Step past the now transmitted char. */
                tx_state = TX_IN_PROGRESS;      /* Change state back to IN_PROGRESS. */
                return KISS_TFESC;              /* Return the transposed char. */
            }
    }
}


/** @brief Check if the transmitter system is ide.
 * @return: 0 if not 1 if yes.
 * @NOTE: This function can be called to check in interrupts for further TX-ready chars should be ignored.
 */
static inline unsigned char tx_is_idle (void) {
    return (tx_state == TX_IDLE);
}


/** @brief  Start transmitting a data packet.
 *
 * @param[in]  ptr      Pointer to the first char of the data packet.
 * @param[in]  len      Number of bytes in the data packet.
 */
static inline void tx_start (void * const ptr, const unsigned char len) {
    tx_length = len;            /* Store the length value in static storage. */
    tx_buffer = ptr;            /* Store the data pointer in static storage. */
    tx_state  = TX_STARTING;    /* Switch state to STARTING. */
}


/** @brief  Static memory for the packet receiver.
 */
static unsigned char    rx_buffer [255];
static unsigned char    rx_length;
static enum {
    RX_HUNT,                    /* Waiting for a start flag. */
    RX_IN_PROGRESS,             /* Receiving data. */
    RX_ESCAPE,                  /* Receiving escaped data byte. */
    RX_COMPLETE                 /* Packet reception complete. */
} rx_state = RX_HUNT;


/** @brief  Reset the packet receiver statemachine for a new packet.
 */
static inline void rx_reset (void) {
    rx_state = RX_HUNT;
}


/** @brief  Service function to be called with a received char.
 *
 * @param[in]  ch   The received char.
 * @return: 0 if reception is still in progress, 1 if a packet have been received.
 */
static unsigned char rx_char (const unsigned char ch) {
    switch (rx_state) {
        case RX_HUNT:
            if (ch == KISS_FEND) {                          /* Here is the start char. Get to work. */
                rx_length = 0;                              /* Reset the byte counter. */
                rx_state = RX_IN_PROGRESS;                  /* Switch state to IN_PROGRESS. */
            }
            return 0;                                       /* Nothing is ready yet. */

        case RX_IN_PROGRESS:                                /* Process a received char. */
            switch (ch) {
                case KISS_FEND:                             /* Here is a closing FEND flag. Process packet. */
                    if (rx_length == 0) {                   /* Zero length packet. Just ignore it. */
                        return 0;
                    }
                    rx_state = RX_COMPLETE;                 /* Mark the reception as complete. */
                    return 1;                               /* Return the Packet-Ready flag. */

                case KISS_FESC:                             /* Here is a FESC flag. */
                    if (rx_length >= sizeof (rx_buffer)) {  /* Problem. The packet is going to be too long. */
                        rx_state = RX_HUNT;                 /* Restart the collector. */
                        return 0;
                    }

                    rx_state = RX_ESCAPE;                   /* Change state to process the escaped char. */
                    return 0;

                default:                                    /* Any other char received. Store it. */
                    if (rx_length >= sizeof (rx_buffer)) {  /* Proble. The packet is too long. */
                        rx_state = RX_HUNT;                 /* Restart the collector. */
                        return 0;
                    }
                    rx_buffer [rx_length] = ch;             /* Store the received char as it is. */
                    rx_length++;                            /* Increment the length counter. */
                    return 0;
            }

        case RX_ESCAPE:                                     /* Process the char following a FESC. */
            switch (ch) {
                case KISS_TFEND:                            /* Received a escaped FEND char. */
                    rx_buffer [rx_length] = KISS_FEND;      /* Store the real char. */
                    rx_length++;
                    rx_state = RX_IN_PROGRESS;              /* Change state back to IN_PROGRESS. */
                    return 0;

                case KISS_TFESC:                            /* Received a escaped FESC char. */
                    rx_buffer [rx_length] = KISS_FESC;      /* Store the real char. */
                    rx_length++;
                    rx_state = RX_IN_PROGRESS;              /* Change state back to IN_PROGRESS. */
                    return 0;

                case KISS_FEND:                             /* Received a regular FEND char. */
                    rx_state = RX_IN_PROGRESS;              /* This could be the start of a new frame. */
                    rx_length = 0;                          /* Restart the collector. */
                    return 0;

                default:                                    /* Invalid escape char combination. */
                    rx_state = RX_HUNT;                     /* Restart the collector. */
                    return 0;
            }

        case RX_COMPLETE:
            return 0;                                       /* The rx_buffer is still being parsed. Ignore the char. */
    }
}

