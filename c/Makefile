all: clean arduprint ardushipper main ardutest

ardushipper:
	gcc ardushipper.c -o ardushipper

arduprint:
	gcc arduprint.c -o arduprint

ardutest:
	gcc ardutest.c -o ardutest

main:
	gcc main.c -o main

clean:
	-@rm main
	-@rm ardushipper
	-@rm arduprint
	-@rm ardutest
