# Timewarp for LibAV

## About

This project adds several audio filters to LibAV:
* **atempo** - Adjust audio tempo
* **volume** - Change input volume
* **timewarp** - Adjust audio tempo dynamically based on commands from UNIX pipe
* **volumewarp** - Change input volume dynamically based on commands from UNIX pipe

The **atempo** and **volume** filters are copied from the FFmpeg project.  They take a floating-point argument specifying how to adjust (respectively) the tempo and volume of an audio source.

The **timewarp** and **volumewarp** use the audio filtering algorithms from **atempo** and **volume**, but instead of applying the same tempo/volume adjustment to the entire audio source, they dynamically adjust the tempo/volume based on commands from another process.  They commands are read from a FIFO pipe using non-blocking I/O.  The name of this pipe specified as a command line argument to the filters. The **timewarp** and **volumewarp** filters expect to read double-precision floating-point numbers from the pipe.

For the **atempo** and **timewarp** filters, the value of the tempo adjustment (passed in as a command line parameter to **atempo** and through a pipe to **timewarp**) must be in the [0.5, 2.0] range.


## Usage

**timewarp**:

       avconv \
           -i rtmp://server/live/input_stream \
           -af timewarp=tempopipe=FIFO_FILENAME \
           -vn \
           -f flv \
           rtmp://server/live/output_stream

**volumewarp**:

       avconv \
           -i rtmp://server/live/input_stream \
           -af volumewarp=gainpipe=FIFO_FILENAME \
           -vn \
           -f flv \
           rtmp://server/live/output_stream


## Installation

    ./configure --enable-libmp3lame --prefix=/usr/local/libav-timewarp
    make
    sudo make install
