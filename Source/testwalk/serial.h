//used for serial open, read write using POSIX. As of configured for data communication between FreindlyARM to
//MF-800
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <unistd.h>
#include <string>
#include <termios.h>
#include <string>
#ifdef WIN32
#include <windows.h>
#endif

namespace HAL
{
  class Serial
  {
    #ifdef WIN32
      HANDLE hCom;
    #else
      int fd;
    #endif
    public:
      Serial()
      {
        #ifdef WIN32
              hCom = INVALID_HANDLE_VALUE;
        #else
              fd = 0;
        #endif
      }
      ~Serial()
      {
        #ifdef WIN32
              if (hCom != INVALID_HANDLE_VALUE)
                Close();
        #else
              if (fd)
                close(fd);
        #endif
      }
      bool Open(const std::string& device, int baud);
      void Close(void);
      int Read(void *buf, int size)
      #ifdef WIN32
        ;
      #else
        {
          return(::read(fd, buf, size));
        }
      #endif
      int Write(void *buf, int size)
      #ifdef WIN32
        ;
      #else
        {
          return(::write(fd, buf, size));
        }
      #endif
      int WriteByte(char b);
      int ReadByte(void);
      void WriteString(std::string s);
      void WriteGen(void *buffer, int nbyte);

  }; // class Serial
} // namespace HAL

#endif
