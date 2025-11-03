BASE = asst4

all: $(BASE)

OS := $(shell uname -s)

ifeq ($(OS), Linux) # Linux 환경 설정
  CPPFLAGS = -I/home/l/i/lib175/usr/glew/include
  LDFLAGS = -L/home/l/i/lib175/usr/glew/lib -L/usr/X11R6/lib
  LIBS = -lGL -lGLU -lglut -lGLEW
  CXX = g++
  CXXFLAGS = -std=c++17
endif

# xcode compiler 설치
# % xcode-select --install
# % sudo xcodebuild -license accept
# % sudo xcode-select --reset # 작동이 안될시

# Homebrew 설치 및 GLEW 설치
# % brew install gcc
# % brew install glew

# Make 사용법
# % make
# % ./asst4

ifeq ($(OS), Darwin) # macOS 환경 설정
  CPPFLAGS = -D__MAC__ -I/usr/local/include -I/opt/homebrew/include
  LDFLAGS = -framework GLUT -framework OpenGL -L/usr/local/lib -L/opt/homebrew/lib
  LIBS = -lGLEW
  CXX = clang++
  CXXFLAGS = -std=c++17
endif

ifdef OPT
  # 최적화 옵션
  CXXFLAGS += -O2
else
  # 디버깅 옵션
  CXXFLAGS += -g
endif

OBJ = $(BASE).o ppm.o glsupport.o scenegraph.o picker.o

$(BASE): $(OBJ)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJ) $(BASE)