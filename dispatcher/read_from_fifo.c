#include <errno.h>       //  errno
#include <fcntl.h>       //  O_RDONLY, O_NONBLOCK
#include <stdio.h>       //  printf()
#include <stdlib.h>      //  exit()
#include <sys/types.h>
#include <sys/stat.h>

#define PIPENAME "metronome.pipe"


int main(int argc, char **argv) {

  char *pipename;
  double volume;
  FILE *pipe;
  int fd;

  if (argc == 2) {
    pipename = argv[1];
  }
  else {
    pipename = PIPENAME;
  }

  if ((fd = open(pipename, O_RDONLY | O_NONBLOCK)) == -1) {
    perror("ERROR");
    exit(-1);
  }

  if ((pipe = fdopen(fd, "r")) == NULL) {
    perror("ERROR");
    exit(-1);
  }

  printf("Listening...\n");

  while (1) {
    if (fread(&volume, sizeof(double), 1, pipe) == 1) {
      printf("Volume = %f\n", volume);
    }
    else if (ferror(pipe) && errno != EAGAIN) {
      perror("ERROR");
      exit(-1);
    }
  }

  return(0);
}
