#include <linux/kernel.h>
#include <linux/sfp.h>
#include "sff.h"

const char *sff_link_len(char *buf, size_t size, unsigned int length,
			 unsigned int multiplier)
{
	if (length == 0)
		return "unsupported/unspecified";

	if (length == 255) {
		*buf++ = '>';
		size -= 1;
		length -= 1;
	}

	length *= multiplier;

	if (length >= 1000)
		snprintf(buf, size, "%u.%0*ukm",
			length / 1000,
			multiplier > 100 ? 1 :
			multiplier > 10 ? 2 : 3,
			length % 1000);
	else
		snprintf(buf, size, "%um", length);

	return buf;
}
EXPORT_SYMBOL_GPL(sff_link_len);

const char *sff_bitfield(char *buf, size_t size,
			 const struct sff_bitfield *bits, unsigned int val)
{
	char *p = buf;
	int n;

	*p = '\0';
	while (bits->mask) {
		if ((val & bits->mask) == bits->val) {
			n = snprintf(p, size, "%s%s",
				     buf != p ? ", " : "",
				     bits->str);
			if (n == size)
				break;
			p += n;
			size -= n;
		}
		bits++;
	}

	return buf;
}
EXPORT_SYMBOL_GPL(sff_bitfield);

const char *sff_connector(unsigned int connector)
{
	switch (connector) {
	case SFF8024_CONNECTOR_UNSPEC:
		return "unknown/unspecified";
	case SFF8024_CONNECTOR_SC:
		return "SC";
	case SFF8024_CONNECTOR_FIBERJACK:
		return "Fiberjack";
	case SFF8024_CONNECTOR_LC:
		return "LC";
	case SFF8024_CONNECTOR_MT_RJ:
		return "MT-RJ";
	case SFF8024_CONNECTOR_MU:
		return "MU";
	case SFF8024_CONNECTOR_SG:
		return "SG";
	case SFF8024_CONNECTOR_OPTICAL_PIGTAIL:
		return "Optical pigtail";
	case SFF8024_CONNECTOR_MPO_1X12:
		return "MPO 1X12";
	case SFF8024_CONNECTOR_MPO_2X16:
		return "MPO 2X16";
	case SFF8024_CONNECTOR_HSSDC_II:
		return "HSSDC II";
	case SFF8024_CONNECTOR_COPPER_PIGTAIL:
		return "Copper pigtail";
	case SFF8024_CONNECTOR_RJ45:
		return "RJ45";
	case SFF8024_CONNECTOR_MXC_2X16:
		return "MXC 2X16";
	default:
		return "unknown";
	}
}
EXPORT_SYMBOL_GPL(sff_connector);

const char *sff_encoding(unsigned int encoding)
{
	switch (encoding) {
	case SFF8024_ENCODING_UNSPEC:
		return "unspecified";
	case SFF8024_ENCODING_8472_64B66B:
		return "64b66b";
	case SFF8024_ENCODING_8B10B:
		return "8b10b";
	case SFF8024_ENCODING_4B5B:
		return "4b5b";
	case SFF8024_ENCODING_NRZ:
		return "NRZ";
	case SFF8024_ENCODING_8472_MANCHESTER:
		return "MANCHESTER";
	default:
		return "unknown";
	}
}
EXPORT_SYMBOL_GPL(sff_encoding);

MODULE_LICENSE("GPL");
