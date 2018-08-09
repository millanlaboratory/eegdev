pkglib_LTLIBRARIES =

AM_CPPFLAGS=$(CPPFLAGS) -I$(top_srcdir)/src/core \
            -I$(top_builddir)/lib -I$(top_srcdir)/lib
AM_LDFLAGS=-module -avoid-version -no-undefined
GNULIB = $(top_builddir)/lib/libgnu.la

if ACT2_SUPPORT
pkglib_LTLIBRARIES += biosemi.la
endif
biosemi_la_SOURCES = biosemi.c
biosemi_la_LIBADD = $(ACT2_LIBS) $(GNULIB)

if XDF_SUPPORT
pkglib_LTLIBRARIES += datafile.la
endif
datafile_la_SOURCES = fileout.c
datafile_la_LIBADD = $(XDF_LIBS) $(GNULIB) $(LIB_CLOCK_FUNCS) $(LTLIBINTL)

if NSKY_SUPPORT
pkglib_LTLIBRARIES += neurosky.la
endif
neurosky_la_SOURCES = neurosky.c
neurosky_la_LIBADD = $(NSKY_LIBS) $(GNULIB)

if GTEC_SUPPORT
pkglib_LTLIBRARIES += gtec.la
endif
gtec_la_SOURCES = gtec.c device-helper.h device-helper.c
gtec_la_LIBADD = $(GTEC_LIBS) $(GNULIB) $(LIB_CLOCK_FUNCS)

if TIA_SUPPORT
pkglib_LTLIBRARIES += tobiia.la
endif
tobiia_la_SOURCES = tobiia.c device-helper.h device-helper.c
tobiia_la_LIBADD = $(TIA_LIBS) $(GNULIB) $(LIBSOCKET) $(GETADDRINFO_LIB)

if BARV_SUPPORT
pkglib_LTLIBRARIES += barv.la
endif
barv_la_SOURCES = barv.c RecorderRDA.h device-helper.h device-helper.c
barv_la_LIBADD = $(BARV_LIBS) $(GNULIB) $(LIBSOCKET) $(GETADDRINFO_LIB)

if NEURONE_SUPPORT
pkglib_LTLIBRARIES += neurone.la
endif
neurone_la_SOURCES = neurone.c device-helper.h device-helper.c
neurone_la_LIBADD = $(NEURONE_LIBS) $(GNULIB) $(LIBSOCKET) $(GETADDRINFO_LIB)


if BBT_SUPPORT
pkglib_LTLIBRARIES += bbt.la
endif
bbt_la_SOURCES = bbt.c device-helper.h device-helper.c
bbt_la_LIBADD = $(BBT_LIBS) $(GNULIB)

if GTECNET_SUPPORT
pkglib_LTLIBRARIES += gtecnet.la
endif
gtecnet_la_SOURCES = gtecnet.c device-helper.h device-helper.c
gtecnet_la_LIBADD = $(GTECNET_LIBS) $(GNULIB)

if LSL_SUPPORT
pkglib_LTLIBRARIES += lsl.la
endif
lsl_la_SOURCES = lsl.c device-helper.h device-helper.c
lsl_la_LIBADD = $(LSL_LIBS) $(GNULIB)

if ARDUINO_SUPPORT
pkglib_LTLIBRARIES += arduino.la
endif
arduino_la_SOURCES = arduino.c device-helper.h device-helper.c
arduino_la_LIBADD = $(ARDUINO_LIBS) $(GNULIB)

if WSDSI_SUPPORT
pkglib_LTLIBRARIES += wsdsi.la
endif
wsdsi_la_SOURCES = wsdsi.c DSI.h DSI_API_Loader.c device-helper.h device-helper.c
wsdsi_la_LIBADD = $(WSDSI_LIBS) $(GNULIB)

if Q20_SUPPORT
pkglib_LTLIBRARIES += q20.la
endif
q20_la_SOURCES = q20.c ftd2xx.h device-helper.h device-helper.c
q20_la_LIBADD = $(Q20_LIBS) $(GNULIB)

if EEGO_SUPPORT
pkglib_LTLIBRARIES += eego.la
endif
eego_la_SOURCES = eego.c device-helper.h device-helper.c
eego_la_LIBADD = $(EEGO_LIBS) $(GNULIB)