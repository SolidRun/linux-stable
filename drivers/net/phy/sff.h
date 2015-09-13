#ifndef SFF_H
#define SFF_H

struct sff_bitfield {
	unsigned int mask;
	unsigned int val;
	const char *str;
};

const char *sff_link_len(char *buf, size_t size, unsigned int length,
			 unsigned int multiplier);
const char *sff_bitfield(char *buf, size_t size,
			 const struct sff_bitfield *bits, unsigned int val);
const char *sff_connector(unsigned int connector);
const char *sff_encoding(unsigned int encoding);
#endif
