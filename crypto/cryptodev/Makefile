ccflags-y := -Icrypto/cryptodev -Iinclude/uapi/linux/
obj-$(CONFIG_CRYPTO_CRYPTODEV) += cryptodev.o
cryptodev-objs := ioctl.o \
		  main.o \
		  cryptlib.o \
		  authenc.o \
		  zc.o \
		  util.o
