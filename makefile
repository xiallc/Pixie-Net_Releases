TARGET = cgitraces.cgi gettraces progfippi runstats cgistats.cgi startdaq coincdaq findsettings acquire\
         cgiwaveforms.cgi clockprog pollcsr avgadc cgiavgtraces.cgi cgireadsettings.cgi cgiwritesettings.cgi cgiprinttraces.cgi
LIBS = -lm 
CFLAGS = -std=c99 -Wall
CXXFLAGS = -Wall -O3 -DNDEBUG   -pthread -std=gnu++98
INCDIRS = -I/usr  -I/usr/include -I/usr/local/include
LINKFLAGS =  -static -static-libstdc++
BOOSTLIBS = -L/usr/local/lib -lboost_date_time -lboost_chrono -lboost_atomic -lboost_program_options -lboost_system -lboost_thread -lrt -pthread

.PHONY: default all clean

default: $(TARGET)
all: default

%.o: %.c 
	gcc  $(CFLAGS) -c $< -o $@

%.o: %.cpp 
	g++  $(CXXFLAGS) $(INCDIRS) -c $< -o $@
	
cgitraces.cgi: cgitraces.o PixieNetDefs.h
	gcc cgitraces.o $(LIBS) -o cgitraces.cgi

gettraces: gettraces.o PixieNetDefs.h
	gcc gettraces.o $(LIBS) -o gettraces

cgiprinttraces.cgi: cgiprinttraces.o  PixieNetDefs.h
	g++ cgiprinttraces.o $(LIBS) -o cgiprinttraces.cgi

progfippi: progfippi.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ progfippi.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o progfippi

runstats: runstats.o PixieNetCommon.o PixieNetDefs.h
	gcc runstats.o PixieNetCommon.o $(LIBS) -o runstats

cgistats.cgi: cgistats.o PixieNetCommon.o PixieNetDefs.h
	gcc cgistats.o PixieNetCommon.o $(LIBS) -o cgistats.cgi

startdaq: startdaq.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ startdaq.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o startdaq

coincdaq: coincdaq.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ coincdaq.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o coincdaq

findsettings: findsettings.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ findsettings.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o findsettings

acquire: acquire.o PixieNetConfig.o PixieNetCommon.o PixieNetDefs.h
	g++ acquire.o PixieNetCommon.o PixieNetConfig.o -rdynamic $(LINKFLAGS) $(LIBS) $(BOOSTLIBS) -o acquire

cgiwaveforms.cgi: cgiwaveforms.o PixieNetCommon.o PixieNetDefs.h
	gcc cgiwaveforms.o PixieNetCommon.o $(LIBS) -o cgiwaveforms.cgi

clockprog: clockprog.o PixieNetCommon.o PixieNetDefs.h
	gcc clockprog.o PixieNetCommon.o $(LIBS) -o clockprog

pollcsr: pollcsr.o PixieNetDefs.h
	gcc pollcsr.o $(LIBS) -o pollcsr

avgadc: avgadc.o PixieNetConfig.o PixieNetDefs.h
	g++ avgadc.o PixieNetConfig.o $(LIBS) -o avgadc

cgiavgtraces.cgi: cgiavgtraces.o  PixieNetConfig.o PixieNetDefs.h
	g++ cgiavgtraces.o PixieNetConfig.o $(LIBS) -o cgiavgtraces.cgi

cgiwritesettings.cgi: cgiwritesettings.o PixieNetDefs.h
	g++ cgiwritesettings.o PixieNetCommon.o  $(LIBS) -o cgiwritesettings.cgi

cgireadsettings.cgi: cgireadsettings.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ cgireadsettings.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o cgireadsettings.cgi

# some unused or custom functions normally not compiled
coincdaqm: coincdaqm.o PixieNetCommon.o PixieNetConfig.o PixieNetDefs.h
	g++ coincdaqm.o PixieNetCommon.o PixieNetConfig.o $(LIBS) -o coincdaqm

clean:
	-rm -f *.o
	-rm -f $(TARGET)
