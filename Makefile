marbletest: marbletest.cc
	g++ -Wall -g -m32 marbletest.cc -I/mingw32/include/SDL2 -I../reactphysics3d/src -L../reactphysics3d/build/lib -L/mingw32/lib -Wl,-subsystem,windows -lmingw32 -lSDL2main -lSDL2 -lglew32 -lopengl32 -lreactphysics3d -mwindows -o marbletest.exe
