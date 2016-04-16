all: clean arduprint ardushipper main

ardushipper:
	gcc ardushipper.c -o ardushipper

arduprint:
	gcc arduprint.c -o arduprint

main:
	gcc main.c -o main

clean:
	-@rm main
	-@rm ardushipper
	-@rm arduprint
