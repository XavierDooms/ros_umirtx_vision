all: run

run: redobjdet.x
	./redobjdet.x

redobjdet.x: redobjdet.cpp redobjdetfx.cpp redobjdet.h
	g++ redobjdet.cpp redobjdetfx.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -o redobjdet.x

redobjdetfx.o: redobjdetfx.cpp redobjdet.h
	g++ redobjdet.cpp redobjdetfx.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -o redobjdetfx.o
