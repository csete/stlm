

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

int main (int argc, char **argv)
{
	float		val;
	int		x;
	int		sample = 0;
	int		pbit = 0;
	uint32_t	sr;
	uint32_t	flag = 0x374FE2DA;
	uint32_t	delta;
	uint8_t		databuf [256];
	int		bufin = 0;
	int		plength = 0;
	int		hunting = 1;

	while (1) {
		x = read (0, &val, sizeof (val));
		if (x <= 0) break;

		/* Shift it into the shift register. */
		sr <<= 1;
		if (val >= 0) {
			sr |= 1;
		}
		pbit++;
		//printf ("Sample: %9d  val: %5.3f  ", sample++, val);

		if (hunting) {
			/* Check for flag match. */
			delta = sr ^ flag;

			if (delta == 0) {
				printf ("\nFlag fundet efter %d  ", pbit);
				pbit = 0;
				hunting = 0;
				bufin = 0;
			}
		} else {
			/* Collect packet. */
			if (pbit > 0 && (pbit & 0x07) == 0) {
				/* Got one more byte in the shift register. */
				databuf [bufin] = sr & 0xFF;
				if (bufin == 0) {
					plength = databuf [0];	/* First byte is the packet length. */
				}
				bufin++;
				if (bufin >= plength) {
					printf ("Len: %3d  seq: %3d  CRC: %04X", databuf [0], databuf [1], (databuf [2]<<8) | databuf [3]);
					printf ("  Packet:");
					for (x = 0; x  < bufin; x++) {
						printf (" %02X", databuf [x]);
					}
					hunting = 1;
				}
			}
		}

		//printf ("\n");
	}
}
