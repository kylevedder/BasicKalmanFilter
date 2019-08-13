CC=clang++
FLAGS=-Wall -Werror

all: main.cc
	$(CC) $(FLAGS) -o main main.cc

format:
	clang-format -i --style=Google *.cc

clean:
	rm main