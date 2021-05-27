/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author:   nanoage.co.uk
 *      Modified: pgo
 *
 *  All routines have been marked "weak" in case already defined in library or elsewhere
 *  But the above wasn't sufficient because the library routines are also marked weak (WHY??)
 */
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>

#undef errno
extern int errno;

/**
 * Overridden by actual routine if present
 */
__attribute__((__weak__))
void console_txChar(int ch) {
   (void)ch;
}

/**
 * Overridden by actual routine if present
 */
__attribute__((__weak__))
int console_rxChar(void) {
   return 0;
}

/**
 *  environ
 *
 *  A pointer to a list of environment variables and their values.
 *  For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

/*
 *  close
 *
 *  Close file
 */
__attribute__((__weak__))
int _close(int file __attribute__((unused))) {
   return -1;
}

/*
 *  execve
 *
 *  Transfer control to a new process. Minimal implementation (for a system without processes):
 */
__attribute__((__weak__))
int _execve(char *name __attribute__((unused)), char **argv __attribute__((unused)), char **env __attribute__((unused))) {
   errno = ENOMEM;
   return -1;
}

/**
 *  fork
 *
 *  Create a new process. Minimal implementation (for a system without processes):
 */
__attribute__((__weak__))
int _fork() {
   errno = EAGAIN;
   return -1;
}

/*
 *  fstat
 *
 *  Status of an open file. For consistency with other minimal implementations in these examples,
 *  all files are regarded as character special devices.
 */
__attribute__((__weak__))
int _fstat(int file __attribute__((unused)), struct stat *st __attribute__((unused))) {
   st->st_mode = S_IFCHR;
   return 0;
}

/**
 *  getpid
 *
 *  Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */
__attribute__((__weak__))
int _getpid() {
   return 1;
}

/**
 *  isatty
 *
 *  Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
__attribute__((weak))  // KSDK defines this elsewhere
int _isatty(int file) {
   switch (file){
   case STDOUT_FILENO:
   case STDERR_FILENO:
   case STDIN_FILENO:
      return 1;
   default:
      //errno = ENOTTY;
      errno = EBADF;
      return 0;
   }
}

/*
 *  kill
 *
 *  Send a signal. Minimal implementation:
 */
__attribute__((__weak__))
int _kill(int pid __attribute__((unused)), int sig __attribute__((unused))) {
   errno = EINVAL;
   return (-1);
}

/*
 *  link
 *
 *   Establish a new name for an existing file. Minimal implementation:
 */
__attribute__((__weak__))
int _link(char *old __attribute__((unused)), char *new __attribute__((unused))) {
   errno = EMLINK;
   return -1;
}

/** lseek
 *
 *  Set position in a file. Minimal implementation:
 */
__attribute__((__weak__))
int _lseek(int file __attribute__((unused)), int ptr __attribute__((unused)), int dir __attribute__((unused))) {
   return 0;
}

/*
 * Used by sbrk
 */
static caddr_t heap_end = NULL;

/**
 *  sbrk
 *
 *   Increase program data space.
 *   Malloc and related functions depend on this
 */
__attribute__((__weak__))
caddr_t _sbrk(int incr) {
   extern char __HeapBase;   /* Defined by the linker */
   extern char __HeapLimit;  /* Defined by the linker */
   caddr_t prev_heap_end;
   caddr_t next_heap_end;

   if (heap_end == NULL) {
      /* First allocation */
      heap_end = &__HeapBase;
   }
   prev_heap_end = heap_end;
   // Round top to 2^3 boundary
   next_heap_end = (caddr_t)(((int)prev_heap_end + incr + 7) & ~7);
   if (next_heap_end > &__HeapLimit) {
      /* Heap and stack collision */
#ifdef DEBUG_BUILD
      __asm__("bkpt");
#endif
      errno = ENOMEM;
      return (caddr_t)-1;
   }
   heap_end = next_heap_end;
   return prev_heap_end;
}

/**
 * stat
 *
 * Status of a file (by name). Minimal implementation.
 *
 */
__attribute__((__weak__))
int _stat(const char *filepath __attribute__((unused)), struct stat *st __attribute__((unused))) {
   st->st_mode = S_IFCHR;
   return 0;
}

/**
 * times
 * Timing information for current process. Minimal implementation:
 */
__attribute__((__weak__))
clock_t _times(struct tms *buf __attribute__((unused))) {
   return 0;
}

/**
 * unlink
 *
 * Remove a file's directory entry. Minimal implementation:
 */
__attribute__((__weak__))
int _unlink(char *name __attribute__((unused))) {
   errno = ENOENT;
   return -1;
}

/**
 *  wait
 *
 *  Wait for a child process. Minimal implementation:
 */
__attribute__((__weak__))
int _wait(int *status __attribute__((unused))) {
   errno = ECHILD;
   return -1;
}

/*
 * cmsis-os optional routine
 */
__attribute__((__weak__))
void os_tmr_call(uint16_t  info __attribute__((unused))) {
   (void)info;
}

/**
 * exit
 *
 * Exit process
 *
 * @param rc - Return code from process
 */
__attribute__((__weak__))
void _exit(int rc __attribute__((unused))) {
   for(;;) {
      /*
       * If you end up here it probably means you fell of the end of main() or
       * failed an assertion!
       *
       * Check console output to see description of failed assertions and
       * check the stack trace in the Debug window to find source of problem.
       */
      __asm__("bkpt");
   }
}

/**
 * read
 *
 * Reads characters from a file.
 * 'libc' subroutines will use this system routine for input from all files, including STDIN
 * Blocks until len characters are read or a '\n' is encountered
 *
 * @param file - File to read from (not used - assumed UART=STDIN)
 * @param ptr  - Pointer to buffer for characters
 * @param len  - Maximum number of characters to read
 *
 * @return -1 on error or the number of characters read
 */
int _usbdm_read(int file, char *ptr, int len) {
   if (file != STDIN_FILENO) {
      errno = EBADF;
      return -1;
   }
   int done=0; // Characters read
   int ch;
   do {
      ch = console_rxChar();
      *ptr++ = (char)ch;
   } while ((++done<len) && (ch != '\n'));
   return done;
}

/*
 *  write
 *
 *  Write a character to a file.
 *  `libc' subroutines will use this system routine for output to all files, including stdout
 *
 *  @return -1 on error or number of bytes sent
 */
int _usbdm_write(int file, char *ptr, int len) {
   int n;
   switch (file) {
   case STDOUT_FILENO: /* stdout */
   case STDERR_FILENO: /* stderr */
      for (n = 0; n < len; n++) {
         if (*ptr == '\n') {
            console_txChar('\r');
         }
         console_txChar(*ptr++);
      }
      break;
   default:
      errno = EBADF;
      return -1;
   }
   return len;
}

