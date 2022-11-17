CC = g++

all:     GmslI2cCfg

GmslI2cCfg:  GmslAction.o GmslI2cCfg.o GmslSequence.o Platform.o main.o
	$(CC) -o GmslI2cCfg GmslAction.o GmslI2cCfg.o GmslSequence.o Platform.o main.o 

GmslAction.o:  GmslAction.cpp 
	$(CC) -c GmslAction.cpp

GmslI2cCfg.o:  GmslI2cCfg.cpp 
	$(CC) -c GmslI2cCfg.cpp

GmslSequence.o:  GmslSequence.cpp 
	$(CC) -c GmslSequence.cpp

Platform.o:  Platform.cpp
	$(CC) -c Platform.cpp

main.o:  main.cpp
	$(CC) -c main.cpp

clean: 
	$(RM) GmslI2cCfg *.o
