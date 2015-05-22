CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

% : %.cpp
	g++ -g $(CFLAGS) $(LIBS) -o /home/mahesh/test/$@ $<
