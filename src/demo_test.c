
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <time.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "tm_reader.h"
#include "serial_reader_imp.h"


bool connected;

TMR_Reader r, *rp;
TMR_TransportListenerBlock tb, *listener;
TMR_TagReadData tag;

void errx(int exitval, const char *fmt, ...);
void serialPrinter(bool tx, uint32_t dataLen, const uint8_t data[],
                   uint32_t timeout, void *cookie);
void stringPrinter(bool tx, uint32_t dataLen, const uint8_t data[],
                   uint32_t timeout, void *cookie);

void runcmd(int argc, char *argv[]);
int parseEpc(const char *arg, TMR_TagData *tag, int *nchars);
int parseFilter(const char *arg, TMR_TagFilter **filterp, int *nchars);
int parseTagop(const char *arg, TMR_TagOp **tagopp, int *nchars);
int parseU32List(const char *arg, TMR_uint32List *list, int *nchars);
int parseU8List(const char *arg, TMR_uint8List *list, int *nchars);
int parseWords(const char *arg, TMR_uint16List *list);
int parseBytes(const char *arg, TMR_uint8List *list);
int parseReadPlan(const char *arg, TMR_ReadPlan *plan, int *nchars);
int parseLockAction(const char *arg, TMR_TagLockAction *action);
void printU8List(TMR_uint8List *list);
void printU32List(TMR_uint32List *list);
void printPortValueList(TMR_PortValueList *list);
void printReadPlan(TMR_ReadPlan *plan);
void printFilter(TMR_TagFilter *filter);
void printTagop(TMR_TagOp *tagop);
const char *listname(const char *list[], int listlen, unsigned int id);
int listid(const char *list[], int listlen, const char *name);

char *command_generator(const char *, int);
char **demo_completion(const char *, int, int);

#define numberof(x) (sizeof((x))/sizeof((x)[0]))


char *getcommand_interactive();
char *getcommand_noninteractive();
char *getcommand_interactive()
{
  return readline(">>> ");
}

int
main(int argc, char *argv[])
{
  char *uri;
  int opt;
  int verbose;
  TMR_Status ret;
  TMR_Region region;
  
  char *saveptr, *arg, *args[16];
  char *line;
  char *(*getcommand)();
  verbose = 0;
  while ((opt = getopt(argc, argv, "v")) != -1)
  {
    switch (opt) {
    case 'v':
      verbose = 1;
      break;
    default: /* '?' */
      fprintf(stderr, "Usage: %s [-v] uri\n",
              argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  
  argc -= optind;
  argv += optind;

  if (argc < 1)
  {
    printf("Usage: demo reader-uri <command> [args]\n"
           "  (URI: 'tmr:///COM1' or 'tmr://astra-2100d3/' "
           "or 'tmr:///dev/ttyS0')\n\n");
    exit(1);
  }
  
  uri = argv[0];
  rp = &r;
  ret = TMR_create(rp, uri);

  if (TMR_SUCCESS != ret)
  {
    //errx(1, "Error creating reader: %s\n", TMR_strerr(rp, ret));
  }

  if (TMR_READER_TYPE_SERIAL == rp->readerType)
  {
    tb.listener = serialPrinter;
  }
  else
  {
    tb.listener = stringPrinter;
  }

  tb.cookie = stdout;
}


void
serialPrinter(bool tx, uint32_t dataLen, const uint8_t data[],
              uint32_t timeout, void *cookie)
{
  FILE *out;
  uint32_t i;

  out = cookie;

  fprintf(out, "%s", tx ? "Sending: " : "Received:");
  for (i = 0; i < dataLen; i++)
  {
    if (i > 0 && (i & 15) == 0)
    {
      fprintf(out, "\n         ");
    }
    fprintf(out, " %02x", data[i]);
  }
  fprintf(out, "\n");
}


void stringPrinter(bool tx,uint32_t dataLen, const uint8_t data[],uint32_t timeout, void *cookie)
{
  FILE *out = cookie;

  fprintf(out, "%s", tx ? "Sending: " : "Received:");
  fprintf(out, "%s\n", data);
}
