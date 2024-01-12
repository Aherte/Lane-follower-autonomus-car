CC = g++
PROJECT = Autonomus_Car
SRC = Autonomus_Car.cpp
INCLUDE_OPENCV = -I/usr/local/include/opencv4/
LIBS_OPENCV = `pkg-config --cflags --libs opencv4`

INCLUDE_RASPICAM = -I/usr/local/include
LIBS_RASPICAM = `pkg-config --cflags --libs raspicam`

INCLUDE_WIRINGPI = -I/usr/local/include
LIBS_WIRINGPI = `pkg-config --cflags --libs wiringpi`

$(PROJECT): $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(INCLUDE_OPENCV) $(LIBS_OPENCV) $(INCLUDE_RASPICAM) $(LIBS_RASPICAM) $(INCLUDE_WIRINGPI) $(LIBS_WIRINGPI)

.PHONY: clean
clean:
	rm -f $(PROJECT)

.PHONY: run
run:
	./$(PROJECT)

.PHONY: upload
upload: clean
	make
	./$(PROJECT)
