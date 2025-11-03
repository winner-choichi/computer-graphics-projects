BASE = asst7

all: $(BASE)

OS := $(shell uname -s)

ifeq ($(OS), Linux) # Science Center Linux Boxes
  CPPFLAGS = -I/home/l/i/lib175/usr/glew/include
  LDFLAGS += -L/home/l/i/lib175/usr/glew/lib -L/usr/X11R6/lib
  LIBS += -lGL -lGLU -lglut
endif

ifeq ($(OS), Darwin) # macOS
  CPPFLAGS = -D__MAC__ -I/usr/local/include -I/opt/homebrew/include
  LDFLAGS = -framework GLUT -framework OpenGL -L/usr/local/lib -L/opt/homebrew/lib
  LIBS = -lGLEW
  CXX = clang++
  CXXFLAGS = -std=c++17
endif


ifdef OPT 
  #turn on optimization
  CXXFLAGS += -O2
else 
  #turn on debugging
  CXXFLAGS += -g
endif

CXX = g++ 

OBJ = $(BASE).o ppm.o glsupport.o scenegraph.o picker.o geometry.o material.o renderstates.o texture.o

$(BASE): $(OBJ)
	$(LINK.cpp) -o $@ $^ $(LIBS) -lGLEW 

clean:
	rm -f $(OBJ) $(BASE)
