marbletest: marbletest.cc
	g++ -Wall -g -m32 marbletest.cc -I/mingw32/include/SDL2 -L/mingw32/lib -Wl,-subsystem,windows -lmingw32 -lSDL2main -lSDL2 -lglew32 -lopengl32 -mwindows -o marbletest.exe
